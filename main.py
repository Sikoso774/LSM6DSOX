import sys
from app import IMU_Application

if __name__ == "__main__":
    app = IMU_Application()
    
    try:
        if not app.initialize():
            sys.exit(1)
        
        app.run(0.5)

    finally:
        app.cleanup()