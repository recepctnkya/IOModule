# Hexnet IO: ESP-IDF default flash baud (460800). Use ESPBAUD=38400 if link is unstable.
import os


def action_extensions(base_actions, project_path=None):
    os.environ.setdefault('ESPBAUD', '460800')
    return {}
