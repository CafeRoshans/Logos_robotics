gfsx_20260715_233505
тут полный рандом, нет рандома

gfsx_20260716_095711
тут каждые 10к, нет рандома

...
  
gfsx_20260716_140735
тут добавил сцены, привел к норме штрафы
Обучение становится более стабильным, но видимо роботы выходят к тактике ехать вдоль стены
стоит поставить меньше безопасной зоны


надо попроботвать curriculum learning с разными этапами обучения скиллов (езда, хват и тд). 
ввести что-то вроде повторения, этапы идущие через несколько этапов обучения скилла, где на новых настройках модель пытается решить тест


попробовать 4. Behavior Cloning (BC) + RL Fine-tuning


1) добавить они добавляют шум прямо в CollectObservations

УЗ: sensor.AddObservation(ultrasonicDist + Random(-0.05, 0.05))
Vision angle: + Random(-noiseAmp, noiseAmp) где noiseAmp читается из environment_parameters.vision_noise
Vision distance: + Random(-noiseAmp*3, noiseAmp*3) (в 3× большая амплитуда для дистанции)

2) убрать grabbing без discrete
3 continuous, без discrete
Gas [-1..1]
Steering [-1..1]
CameraYaw [-1..1]
Gripper управляется ПРОГРАММНО, не сетью. Их GripperController закрывается автоматически когда gripperIR = 1.

Наш: 3 continuous + 1 discrete
Gas, Steering, CameraYaw
DiscreteActions[0] = 0/1/2 → grab/release решение сети


gfsx_20260717_101159/GFSX_Brain
