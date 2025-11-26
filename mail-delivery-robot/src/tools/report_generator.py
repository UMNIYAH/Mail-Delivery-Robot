import os
from datetime import datetime
from jinja2 import Environment, FileSystemLoader


class ReportGenerator:

    def __init__(self, template_path):
        template_dir = os.path.dirname(template_path)
        template_name = os.path.basename(template_path)

        env = Environment(loader=FileSystemLoader(template_dir))
        try:
            self.template = env.get_template(template_name)
        except Exception as e:
            raise RuntimeError(f"Failed to load template: {e}")

    def generate_report(self, output_dir, **data):
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"report_{timestamp}.html"
        output_path = os.path.join(output_dir, filename)

        html_content = self.template.render(**data)

        with open(output_path, "w") as f:
            f.write(html_content)

        return output_path
