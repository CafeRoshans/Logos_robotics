using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // Требуется для TwistMsg
using RosMessageTypes.Std;      // Требуется для Int32Msg и Float32Msg

public class ROSBridge : MonoBehaviour
{
    public string topicName = "/cmd_vel";
    public float maxLinearSpeed = 0.5f;   // Линейный лимит реального робота (м/с)
    public float maxAngularSpeed = 1.0f;  // Угловой лимит реального робота (рад/с)

    [Range(0.1f, 1f)]
    public float emaAlpha = 0.8f;         // Коэффициент сглаживания (0.8 = высокая отзывчивость)

    private ROSConnection ros;
    private float smoothGas = 0f;
    private float smoothSteering = 0f;

    void Start()
    {
        // Получаем экземпляр ROS-подключения
        ros = ROSConnection.GetOrCreateInstance();
        
        // Регистрируем топики для публикации
        ros.RegisterPublisher<TwistMsg>(topicName);
        ros.RegisterPublisher<Int32Msg>("/cmd_gripper");
        ros.RegisterPublisher<Float32Msg>("/cmd_camera_pan");
    }

    // Метод отправки сглаженных скоростей в /cmd_vel
    public void PublishCommand(float gas, float steering)
    {
        if (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steering, 0f))
        {
            smoothGas = 0f;
            smoothSteering = 0f;
        }
        else
        {
            smoothGas = emaAlpha * gas + (1f - emaAlpha) * smoothGas;
            smoothSteering = emaAlpha * steering + (1f - emaAlpha) * smoothSteering;
        }

        TwistMsg cmd = new TwistMsg();
        cmd.linear.x = smoothGas * maxLinearSpeed;
        cmd.angular.z = smoothSteering * maxAngularSpeed;

        ros.Publish(topicName, cmd);
    }

    // Метод отправки команд манипулятора в /cmd_gripper
    public void PublishGripperCmd(int cmd)
    {
        Int32Msg msg = new Int32Msg();
        msg.data = cmd;
        ros.Publish("/cmd_gripper", msg);
    }

    // Метод отправки угла камеры в /cmd_camera_pan
    public void PublishCameraCmd(float yaw)
    {
        Float32Msg msg = new Float32Msg(yaw);
        ros.Publish("/cmd_camera_pan", msg);
    }
}
