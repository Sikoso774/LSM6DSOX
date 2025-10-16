import sys
from app import IMU_Application

if __name__ == "__main__":
    # 1. Créer l'application
    app = IMU_Application()
    
    try:
        # 2. Initialiser le capteur
        if not app.initialize():
            # L'erreur a déjà été affichée par initialize()
            sys.exit(1)
        
        # 3. Lancer la boucle principale (le cœur de l'application)
        app.run()

    finally:
        # 4. Nettoyage
        app.cleanup()