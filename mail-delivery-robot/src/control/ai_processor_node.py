import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ollama import chat

"""
ros2 run captain
ros2 run ai_processor_node
"""
class OllamaChat:
    def __init__(self, model = "qwen3", stream = False, think = False):
        self.model = model
        self.stream = stream
        self.think = think

    def ask(self, prompt: str) -> str:
        try:
            response = chat(
                model = self.model,
                messages = [{"role" : "user", "text" : prompt}],
                think = self.think,
                stream = self.stream,
            )
            return response.messages.content.strip()
        except Exception as e:
            return f"Error: {prompt}"

class ai_processor_node(Node):
    """
    general purpose AI Processor Node, subscribes to multiple topics
    basically send data to LLM and publish responses
    If you want you can have each topic have its own prompt template and output topic

    """
    def __init__(self):
        super().__init__("ai_processor_node")

        self.model = "qwen3"
        self.think = False
        self.stream = False
        # can make your own config
        # input = topic to sub
        # output = topic to pub to AI
        # prompt = your prompt for the LLM
        self.topics = [
            {
                "input": "/battery_status_text",
                "output": "/ai/battery_advice",
                "prompt": "Battery report: {data}. Summarize what the robot should do next."
            },
            {
                "input": "/captain_status",
                "output": "/ai/captain_advice",
                "prompt": "Here are the current robot actions: {data}. Suggest the best next action."
            }
        ]

        self.ollama = OllamaChat(self.model, think = self.think, stream = self.stream)
        self.publishers = {}

        for topic in self.topics:
            input_topic = topic["input"]
            output_topic = topic["output"]
            prompt_template = topic["prompt"]

            pub = self.create_publisher(String, output_topic, 10)
            self.publishers[input_topic] = pub

            self.create_subscription(
                String,
                input_topic,
                lambda msg, it=input_topic, pt=prompt_template: self.process_message(it, pt, msg),
                10
            )
            self.get_logger().info(f"Listening on {input_topic}, publishing to {output_topic}")

    def process_message(self, topic_name: str, prompt_template: str, msg: String):
        text = msg.data.strip()
        prompt = prompt_template.format(data = text)

        self.get_logger().info(f"[{topic_name}] Prompt: {prompt}")
        response = self.ollama.ask(prompt)

        out_msg = String()
        out_msg.data = response
        self.publishers[topic_name].publish(out_msg)
        self.get_logger().info(f"[{topic_name}] AI Response: {response}")

def main(args=None):
    rclpy.init(args=args)
    node = ai_processor_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
