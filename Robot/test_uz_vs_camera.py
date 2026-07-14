#!/usr/bin/env python3
"""
Диагностика: УЗ датчик vs угол камеры.
Крутим камеру от 0° до 180° с шагом 10° и читаем ультразвук.
Если УЗ показывает < 30cm на каком-то угле — значит луч бьёт в плату/конструкцию.

Запуск: python3 /root/test_uz_vs_camera.py
(или через docker exec)
"""
import sys
import time
sys.path.append('/root/XiaoRGeek')

from xr_ultrasonic import Ultrasonic
import xr_ultrasonic
servo = xr_ultrasonic.servo

us = Ultrasonic()

print("=" * 50)
print("ДИАГНОСТИКА: УЗ датчик vs угол камеры")
print("=" * 50)
print(f"{'Угол камеры':>15} | {'УЗ дист (cm)':>15} | {'Статус':>10}")
print("-" * 50)

problem_angles = []

for angle in range(0, 181, 10):
    servo.set(7, angle)  # SERVO_CAMERA_PAN = 7
    time.sleep(0.5)  # Ждём пока серво дойдёт
    
    # Читаем 3 замера и берём медиану
    readings = []
    for _ in range(3):
        d = us.get_distance()
        if d <= 0 or d > 500:
            d = 500.0
        readings.append(d)
        time.sleep(0.05)
    
    median_cm = sorted(readings)[1]
    
    status = "✅ OK"
    if median_cm < 30:
        status = "🔴 ПЛАТА?!"
        problem_angles.append((angle, median_cm))
    elif median_cm < 60:
        status = "⚠️ БЛИЗКО"
    
    print(f"{angle:>12}° | {median_cm:>12.1f}cm | {status:>10}")

# Возвращаем камеру в центр
servo.set(7, 90)

print("=" * 50)
if problem_angles:
    print(f"⚠️ ПРОБЛЕМНЫЕ УГЛЫ ({len(problem_angles)}):")
    for ang, dist in problem_angles:
        print(f"   Угол {ang}° → УЗ = {dist:.1f}cm (попадает в конструкцию!)")
    
    # Рекомендация по ограничению
    safe_min = min(a for a, _ in problem_angles)
    safe_max = max(a for a, _ in problem_angles)
    print(f"\n💡 РЕКОМЕНДАЦИЯ: ограничить камеру до [{safe_min+10}°..{safe_max-10}°]")
    print(f"   В нормализованном виде (Unity): [{(90-(safe_max-10))/90:.2f}..{(90-(safe_min+10))/90:.2f}]")
else:
    print("✅ УЗ датчик не ловит конструкцию ни на одном угле камеры.")

print("Камера возвращена в центр (90°)")
