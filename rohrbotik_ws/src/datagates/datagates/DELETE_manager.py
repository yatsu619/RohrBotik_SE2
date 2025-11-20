from cam_VisionProcessor_logic import VisionProcessor



class VisionManager:
    """
    Erstellt eine einzige Instanz des VisionProcessors und lässt mehrer einzelne Programme darauf zugreifen
    
    Für das Nutzen dieser Instanz (und damit auch die Variablen des Objekts "VisionProcessor"),
    muss in jeder auf den VP zugreifende Code folgendes importieren: from vision_manager import VisionManager
    
    
    Gründe für einen Manager:
<<<<<<<<<<<<<<<<<<<<<< Grundlegend ist es für dieses Kleine Projekt overkilled, aber es gibt Gründe es gleich "richtig" und im "Singleton" zu machen. >>>>>>>>>>>>>>>>>>>>>>

        1. Zirkuläre Impoorts schwer zu debuggen (hier gibt es eine klare Hirachie und die Imports sind sauber getrennt:  Manager importieren nur Visionporzessor // Nodes nur Manager)
        2. Testen wird schwerer (mit Manager kann ein Mock verwendet werden) 
        3. Wird erst dann erstellt, wenn er wirklich gebraucht wird. Kein Lazy Loading! (ArUco-Dictionary wird direkt beim Import geladen // Speicher wird alloziert direkt beim Import)

<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    """

    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = VisionProcessor()
        return cls._instance
