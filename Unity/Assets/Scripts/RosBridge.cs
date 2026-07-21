using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // Требуется для TwistMsg
using RosMessageTypes.Std;      // Требуется для Int32Msg и Float32Msg

public class ROSBridge : MonoBehaviour
{
    public string driveTopic = "/cmd_vel";
    public string gripperTopic = "/cmd_gripper";
    public string cameraTopic = "/cmd_camera_pan";

    public float maxLinearSpeed = 0.5f;   // Линейный лимит реального робота (м/с) ; как будто где-то конфликт с другими модулями
    public float maxAngularSpeed = 1.0f;  // Угловой лимит реального робота (рад/с)

    [Range(0.1f, 1f)]
    public float emaAlpha = 0.8f;         // Коэффициент сглаживания (0.8 = высокая отзывчивость)

    private ROSConnection ros;

    public System.DateTime lastCommandPublishTime;

    void Start()
    {
        // Получаем экземпляр ROS-подключения
        ros = ROSConnection.GetOrCreateInstance();
        
        // Регистрируем топики для публикации
        ros.RegisterPublisher<TwistMsg>(driveTopic);
        ros.RegisterPublisher<Int32Msg>(gripperTopic);
        ros.RegisterPublisher<Float32Msg>(cameraTopic);
    }

    // Метод отправки сглаженных скоростей в /cmd_vel
    public void PublishCommand(float gas, float steering)
    {
        // EMA отключён так как сглаживание уже есть на Pi (unity_master_team2.py, EMA_STEER=0.40).
        // короче депрекате

        TwistMsg cmd = new TwistMsg();
        cmd.linear.x = gas * maxLinearSpeed;
        cmd.angular.z = steering * maxAngularSpeed;

        if (lastCommandPublishTime != null && (System.DateTime.Now - lastCommandPublishTime).TotalSeconds > 0.5)
        {
            ros.Publish(driveTopic, cmd);
            Debug.LogWarning("ROSBridge: last command was published more than 0.5 seconds ago. Check ROS connection.");

        }

        ros.Publish(driveTopic, cmd);
    }

    // Метод отправки команды манипулятора в /cmd_gripper
    public void PublishGripperCmd(int cmd)
    {
        Int32Msg msg = new Int32Msg();
        msg.data = cmd;
        ros.Publish(gripperTopic, msg);
    }

    // Метод отправки угла камеры в /cmd_camera_pan
    public void PublishCameraCmd(float yaw)
    {
        Float32Msg msg = new Float32Msg(yaw);
        ros.Publish(cameraTopic, msg);

    }
}
