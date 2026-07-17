using UnityEngine;
using System;
using System.Threading;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Collections.Concurrent;


[System.Serializable]
public class YoloDataPacket
{
    public float angle;      // Отклонение центра мяча (-1.0 лево, 1.0 право)
    public float distance;   // Высота рамки мяча относительно кадра (0..1)
    public float sees;       // Флаг видимости (1.0 = виден, 0.0 = нет)
    public float conf;       // Уверенность детекции
    public float w;          // Ширина bounding box
    public float h;          // Высота bounding box
}


public class RealVision : MonoBehaviour
{
    public int udpPort = 5005;
    public bool useYOLO = false;
    
    // Телеметрия для RobotBrain
    public float normalizedAngle;
    public float normalizedDistance;
    public bool seesBall;

    private CancellationTokenSource cts;
    private ConcurrentQueue<YoloDataPacket> udpQueue = new ConcurrentQueue<YoloDataPacket>();

    [HideInInspector] public bool  isVisible = false;

    [HideInInspector] public float horizontalAngle = 0f;

    void Start()
    {
        cts = new CancellationTokenSource();
        // Запуск UDP-слушателя в фоновом потоке, чтобы не вешать игру
        Task.Run(() => UdpListenerLoop(cts.Token));
    }

    private async Task UdpListenerLoop(CancellationToken token)
    {
        using (var udpClient = new UdpClient(udpPort))
        {
            while (!token.IsCancellationRequested)
            {
                var result = await udpClient.ReceiveAsync();
                string json = System.Text.Encoding.UTF8.GetString(result.Buffer);
                
                YoloDataPacket packet = JsonUtility.FromJson<YoloDataPacket>(json);
                if (packet != null)
                {
                    udpQueue.Enqueue(packet); // Безопасно кладем в очередь
                }
            }
        }
    }

    void Update()
    {
        // Читаем пакеты из очереди на главном потоке Unity
        while (udpQueue.TryDequeue(out var packet))
        {
            useYOLO = true;
            seesBall = packet.sees > 0.5f;

            if (seesBall)
            {
                // Ограничиваем угол [-1, 1]
                normalizedAngle = Mathf.Clamp(packet.angle, -1f, 1f);
                
                // Записываем нормализованную высоту рамки (чем больше мяч, тем он ближе)
                normalizedDistance = packet.distance; 

                isVisible = true;

                horizontalAngle = packet.angle;
            }
            else
            {
                normalizedAngle = 0f;
                normalizedDistance = 1f; // 1.0 = далеко/мяч не виден
            }
        }
    }

    void OnDestroy()
    {
        cts?.Cancel(); // Останавливаем фоновый поток при выходе
    }
}

