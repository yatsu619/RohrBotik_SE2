from cam_VisionProcessor_logic import VisionProcessor



class VisionManager:
    """
    Erstellt eine einzige Instanz des VisionProcessors und lässt mehrer einzelne Programme darauf zugreifen
    
    Für das Nutzen dieser Instanz (und damit auch die Variablen des Objekts "VisionProcessor"),
    muss in jeder auf den VP zugreifende Code folgendes importieren: from vision_manager import VisionManager
    
    """

    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = VisionProcessor()
        return cls._instance
