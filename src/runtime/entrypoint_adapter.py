# SteerPy author, 2026.

import inspect
import types


class SimulationEntrypointAdapter:
    def __init__(self, function_name="simulate_one_step"):
        self.function_name = function_name

    def resolve_callable(self, namespace):
        update_obj = namespace.get(self.function_name)
        if update_obj is None or not callable(update_obj):
            raise Exception(f"No {self.function_name}(...) found.")
        return update_obj

    @staticmethod
    def _positional_parameters(fn):
        return [
            p
            for p in inspect.signature(fn).parameters.values()
            if p.kind
            in (
                inspect.Parameter.POSITIONAL_ONLY,
                inspect.Parameter.POSITIONAL_OR_KEYWORD,
            )
        ]

    def build_adapter(self, update_obj):
        params = self._positional_parameters(update_obj)
        arity = len(params)

        if arity not in (2, 3):
            raise Exception(
                f"{self.function_name} signature must be "
                f"{self.function_name}(car, world_model) "
                f"or {self.function_name}(car, sensors, world_model)."
            )

        def update_adapter(car, sensors, world_model):
            if isinstance(world_model, dict):
                world_model = types.SimpleNamespace(**world_model)
            if arity == 2:
                return update_obj(car, world_model)
            return update_obj(car, sensors, world_model)

        return update_adapter, arity


_entry_fn_name = "simulate_one_step"
_entrypoint_adapter = SimulationEntrypointAdapter(_entry_fn_name)
_update_obj = _entrypoint_adapter.resolve_callable(globals())
_update_adapter, _update_arity = _entrypoint_adapter.build_adapter(_update_obj)
