
import sys
import os

# Configuration du chemin
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_DIR)

def print_banner():
    """Affiche la bannière de démarrage"""
    print("\n" + "=" * 60)
    print("  ROBOT TURTLEBOT3 - NAVIGATION AUTONOME")
    print("=" * 60)
    print("\n Séquence du parcours :")
    print("   • Balise 5 → Tourner à DROITE")
    print("   • Balise 6 → ACCÉLÉRER")
    print("   • Balise 7 → Tourner à DROITE")
    print("   • Balise 8 → Tourner à DROITE")
    print("   • Balise 9 → STOP FINAL")
    print("\n  Centrage balise : ACTIVÉ")
    print("  Évitement LiDAR : ACTIVÉ")
    print(" Démarrage...\n")

if __name__ == '__main__':
    from navigation import main  #  Importe depuis navigation.py
    
    print_banner()
    
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n Arrêt manuel (Ctrl+C)")
    except Exception as e:
        print(f"\n Erreur : {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n✅ Programme terminé\n")