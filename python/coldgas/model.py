"""Twin structure constants (spec/vehicle.json `twin`), re-exported for the Python twin."""
from .spec import MODEL

MODEL_ID: str = MODEL['model']
CONTACT = MODEL['contact']
PLACEMENT = MODEL['placement']
LIMITS = MODEL['limits']
