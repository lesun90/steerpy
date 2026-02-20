# SteerPy author, 2026.

class ModelLoader:
    def __init__(self, step_name="step", state_name="State"):
        self.step_name = step_name
        self.state_name = state_name

    @staticmethod
    def _build_state_fallback():
        class StateFallback:
            def __init__(self):
                # Runtime fields are injected from simulator each frame.
                pass

        return StateFallback

    def load(self, source_code):
        namespace = {}
        exec(source_code, namespace, namespace)

        step_fn = namespace.get(self.step_name)
        if step_fn is None or not callable(step_fn):
            raise Exception("No function step(state, dt) found.")

        state_cls = namespace.get(self.state_name)
        if state_cls is None:
            state_cls = self._build_state_fallback()

        state_obj = state_cls()
        return namespace, step_fn, state_obj


_model_loader = ModelLoader()
_model_ns, _step_obj, _state_obj = _model_loader.load(_model_src)
