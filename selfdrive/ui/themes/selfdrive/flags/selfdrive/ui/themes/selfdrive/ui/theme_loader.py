from selfdrive.flags.abe_features import AbeFeatures
from selfdrive.ui.themes import tesla_theme

def load_theme():
    if AbeFeatures.TESLA_THEME:
        return tesla_theme.theme
    else:
        # Default theme
        return {}
