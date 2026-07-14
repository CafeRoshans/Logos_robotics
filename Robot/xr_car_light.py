#!/usr/bin/env python3
# Заглушка для xr_car_light — модуль отсутствует в Docker-образе,
# но xr_ultrasonic -> xr_socket -> xr_car_light импортирует его при загрузке.
# Без этого файла ультразвуковой датчик не инициализируется.

class Car_light:
    """Пустой класс-заглушка. Реальные LED-ленты робота нам не нужны."""
    def __init__(self, *args, **kwargs):
        pass
    def set_color(self, *args, **kwargs):
        pass
    def turn_off(self, *args, **kwargs):
        pass
    def turn_on(self, *args, **kwargs):
        pass
    def __getattr__(self, name):
        """Любой несуществующий метод просто ничего не делает."""
        return lambda *a, **kw: None
