#!/usr/bin/env python3
# Заглушка для xr_music — модуль отсутствует в Docker-образе,
# но xr_ultrasonic -> xr_socket -> xr_music импортирует его.

class Beep:
    """Пустой класс-заглушка для пищалки."""
    def __init__(self, *args, **kwargs):
        pass
    def __getattr__(self, name):
        return lambda *a, **kw: None
