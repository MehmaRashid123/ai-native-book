import json
from typing import Dict, Any


def validate_gpu(gpu_model: str, ram_gb: int) -> str:
    """
    Goal: Secure Bonus Points for Personalization.
    Function: validate_gpu(gpu_model: str, ram_gb: int)
    Logic:
      - If gpu_model does not contain "RTX" or "A10", return a WARNING that "NVIDIA Isaac Sim requires RTX GPU."
      - If gpu_model implies MacOS (e.g., "M1", "M2", "Apple"), return a CRITICAL error that "Isaac Sim does not support MacOS."
      - If ram_gb < 32, warn about potential crashes.
    Output: Return a JSON string with status (PASS/WARN/FAIL) and message.
    """
    status = "PASS"
    message = "GPU configuration is suitable for Isaac Sim"

    # Check if GPU is suitable for Isaac Sim
    if not ("RTX" in gpu_model.upper() or "A10" in gpu_model.upper()):
        status = "WARN"
        message = "NVIDIA Isaac Sim requires RTX GPU."

    # Check if it's a MacOS system
    if any(keyword in gpu_model.lower() for keyword in ["m1", "m2", "apple"]):
        status = "FAIL"
        message = "Isaac Sim does not support MacOS."

    # Check RAM
    if ram_gb < 32:
        if status == "PASS":
            status = "WARN"
            message = f"{message} However, {ram_gb}GB RAM may cause crashes during Isaac Sim operations."
        else:
            message = f"{message} Additionally, {ram_gb}GB RAM may cause crashes during Isaac Sim operations."

    result = {
        "status": status,
        "message": message
    }

    return json.dumps(result)