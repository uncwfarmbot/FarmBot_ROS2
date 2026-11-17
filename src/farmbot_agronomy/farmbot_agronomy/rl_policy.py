"""
The PPO controller, run from a policy exported to ONNX.

The policy was trained in the Gymnasium basil environment on ten observations and two
continuous actions (water, nitrogen). Its observation order and normalisation are whatever the
training environment used, so both are configuration here, not code: list them in
rl.observation in agronomy.yaml in exactly the training order. A feature this bed cannot measure
yet (greenness before the OAK-D pipeline exists) is filled from rl.defaults and reported in the
decision, so a policy running on made-up inputs is never silent about it.
"""
from farmbot_agronomy.strategies import Decision, Strategy


class RlPolicy(Strategy):
    """Run an exported PPO actor on each plant's observation."""

    name = 'rl'
    needs_reading = True

    def __init__(self, params: dict, state: dict):
        """Load the ONNX policy. onnxruntime is only needed when this strategy is chosen."""
        super().__init__(params, state)
        import numpy as np
        import onnxruntime

        self.np = np
        self.session = onnxruntime.InferenceSession(params['policy_path'])
        self.input_name = self.session.get_inputs()[0].name

    def features(self, obs, ctx) -> dict:
        """Every observation feature this bed can compute, in physical units."""
        soil = ctx.soil
        f = {
            'season_progress': obs.days_since_planting / self.params['season_days'],
            'air_temp_c': (ctx.t_min + ctx.t_max) / 2,
            'time_of_day': ctx.now.hour / 24.0,
        }
        if obs.reading is not None:
            depletion = soil.depletion_mm(obs.reading['moisture_pct'])
            f.update({
                'soil_water': 1.0 - depletion / soil.taw_mm,
                'soil_n_mg': soil.root_zone_mg(obs.reading['nitrogen_mg_kg']),
                'soil_p_mg': soil.root_zone_mg(obs.reading['phosphorus_mg_kg']),
                'soil_k_mg': soil.root_zone_mg(obs.reading['potassium_mg_kg']),
                'ph': obs.reading['ph'],
                'soil_temp_c': obs.reading['soil_temp_c'],
            })
        return f

    def decide(self, obs, ctx):
        """Normalise the observation, run the actor, and scale its actions to mL and mg."""
        if obs.reading is None:
            return Decision(reason='no valid reading, skipped')
        available = self.features(obs, ctx)
        vector, defaulted = [], []
        for name, offset, scale in self.params['observation']:
            if name in available:
                value = available[name]
            else:
                value = self.params['defaults'][name]
                defaulted.append(name)
            vector.append((value - offset) / scale)

        x = self.np.asarray([vector], dtype=self.np.float32)
        action = self.session.run(None, {self.input_name: x})[0][0]
        low, high = self.params['action_low'], self.params['action_high']
        fractions = [min(max((a - low) / (high - low), 0.0), 1.0) for a in action[:2]]

        reason = f'policy action {[round(float(a), 3) for a in action[:2]]}'
        if defaulted:
            reason += f"; defaulted inputs: {', '.join(defaulted)}"
        return Decision(water_ml=fractions[0] * self.params['max_water_ml'],
                        n_mg=fractions[1] * self.params['max_n_mg'],
                        reason=reason, extra={'observation': vector})
