# SteerPy author, 2026.

class CarConfigLoader:
    def __init__(self, config_symbol="car_config"):
        self.config_symbol = config_symbol

    def load(self, source_code):
        namespace = {}
        exec(source_code, namespace, namespace)

        config_obj = namespace.get(self.config_symbol)
        if config_obj is None:
            raise Exception("No dict 'car_config' found.")

        return namespace, config_obj


_car_config_loader = CarConfigLoader()
_cfg_ns, _cfg_obj = _car_config_loader.load(_cfg_src)
