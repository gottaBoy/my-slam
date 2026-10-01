import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeachText

# 原书直接 `import espeakng`。本机环境（aarch64 容器）没有 espeak-ng 二进制，
# 也没有音频设备（/dev/snd 未映射），安装 espeakng 也没有意义。
# 这里做「优雅降级」：服务名字、类型、返回值都不变，只是不真的发声，改为打印日志。
# 以后要真发声，装上 espeak-ng + 映射音频设备即可自动生效，无需改代码。
try:
    import espeakng
    _HAS_ESPEAKNG = True
except ImportError:
    espeakng = None
    _HAS_ESPEAKNG = False


class Speaker(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.speech_service = self.create_service(
            SpeachText, 'speech_text', self.speak_text_callback)
        if _HAS_ESPEAKNG:
            self.speaker = espeakng.Speaker()
            self.speaker.voice = 'zh'
            self.get_logger().info('语音引擎 espeakng 已就绪（voice=zh）')
        else:
            self.speaker = None
            self.get_logger().warn(
                '未安装 espeakng（或容器无音频设备），进入「只打日志」模式：'
                'speech_text 服务仍然可用，仅打印待朗读文本。')

    def speak_text_callback(self, request, response):
        self.get_logger().info('正在朗读 %s' % request.text)
        if self.speaker is not None:
            self.speaker.say(request.text)
            self.speaker.wait()
        response.result = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Speaker('speaker')
    rclpy.spin(node)
    rclpy.shutdown()