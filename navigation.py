import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from dt_apriltags import Detector
import numpy as np

class NavSimple(Node):
    def __init__(self):
        super().__init__('nav_simple')
        
        # PARAMÈTRES SIMPLES
        self.taille_balise_reelle = 0.17
        self.focale_camera = 320
        self.distance_arret = 0.58  # Distance à laquelle exécuter les commandes
        
        # ROS
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11')
        
        self.balises_vues = []
        self.en_virage = False
        self.debut_virage = 0
        self.vitesse_cible = 0.6  # Vitesse désirée par la vision
        
        # Données LiDAR
        self.distance_gauche = 999
        self.distance_droite = 999
        self.distance_avant = 999
        
        # ZONES DE SÉCURITÉ LIDAR (anticipation progressive)
        self.ZONE_LIBRE = 0.80       # > 80cm : aucune correction
        self.ZONE_VIGILANCE = 0.60   # 60-80cm : correction douce
        self.ZONE_DANGER = 0.40      # 40-60cm : correction + ralentissement
        self.ZONE_CRITIQUE = 0.35    # < 35cm : arrêt
        
        # SATURATIONS (limites de correction)
        self.CORRECTION_MAX = 0.15   # Correction angulaire maximale
        self.VITESSE_MIN = 0.15      # Vitesse minimale (ne jamais s'arrêter sauf critique)
        
        # PARAMÈTRES CENTRAGE BALISE
        self.SEUIL_CENTRE = 30       # Pixels de tolérance (zone "centrée")
        self.GAIN_CENTRAGE = 0.002   # Sensibilité du centrage
        
        self.get_logger().info(' Robot démarré - Centrage balise + sécurité LiDAR')

    def scan_callback(self, msg):
        """Callback LiDAR - Met à jour les distances aux obstacles"""
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 999, ranges)
        
        #  Zones élargies pour anticiper (60-70° de chaque côté)
        self.distance_avant = min(
            np.min(ranges[0:45]),      # 45° devant à droite
            np.min(ranges[315:359])    # 45° devant à gauche
        )
        
        self.distance_droite = np.min(ranges[250:330])  # 80° à droite
        self.distance_gauche = np.min(ranges[30:110])   # 80° à gauche

    def calculer_distance(self, tag):
        """Calcule la distance en mètres à la balise"""
        corners = tag.corners
        largeur = np.linalg.norm(corners[0] - corners[1])
        hauteur = np.linalg.norm(corners[1] - corners[2])
        taille_pixels = (largeur + hauteur) / 2
        
        if taille_pixels > 0:
            return (self.taille_balise_reelle * self.focale_camera) / taille_pixels
        return 999

    def calculer_offset_horizontal(self, tag, largeur_image):
        """
        Calcule l'offset de la balise par rapport au centre de l'image
        Retourne : offset en pixels (négatif = gauche, positif = droite)
        """
        # Centre de la balise
        center_x = np.mean(tag.corners[:, 0])
        
        # Centre de l'image
        image_center = largeur_image / 2
        
        # Offset (+ = droite, - = gauche)
        offset = center_x - image_center
        
        return offset

    def calculer_correction_lidar_progressive(self):
        """
        ✅ Calcule une correction PROGRESSIVE et BORNÉE basée sur le LiDAR
        Retourne : (correction_angulaire, facteur_vitesse)
        """
        correction = 0.0
        facteur_vitesse = 1.0  # Multiplicateur de vitesse (1.0 = vitesse normale)
        
        # ========== OBSTACLE DEVANT (priorité absolue) ==========
        if self.distance_avant < self.ZONE_CRITIQUE:
            self.get_logger().warn('🛑 OBSTACLE CRITIQUE DEVANT', throttle_duration_sec=0.3)
            return 0.0, 0.0  # Arrêt total
        
        elif self.distance_avant < self.ZONE_DANGER:
            facteur_vitesse = 0.4  # Ralentir fortement
            self.get_logger().warn(f'⚠️  Obstacle DEVANT {self.distance_avant:.2f}m', throttle_duration_sec=0.5)
        
        elif self.distance_avant < self.ZONE_VIGILANCE:
            facteur_vitesse = 0.7  # Ralentir modérément
        
        # ========== CORRECTION LATÉRALE PROGRESSIVE ==========
        
        # --- Côté DROIT ---
        if self.distance_droite < self.ZONE_CRITIQUE:
            correction = self.CORRECTION_MAX  # Correction maximale à gauche
            facteur_vitesse = min(facteur_vitesse, 0.3)
            self.get_logger().warn(f'🚨 CRITIQUE DROITE {self.distance_droite:.2f}m', throttle_duration_sec=0.3)
        
        elif self.distance_droite < self.ZONE_DANGER:
            # ✅ Correction proportionnelle (plus proche = plus de correction)
            ratio = 1.0 - (self.distance_droite - self.ZONE_CRITIQUE) / (self.ZONE_DANGER - self.ZONE_CRITIQUE)
            correction = ratio * self.CORRECTION_MAX * 0.8  # 80% de correction max
            facteur_vitesse = min(facteur_vitesse, 0.6)
            self.get_logger().info(f' DANGER DROITE {self.distance_droite:.2f}m', throttle_duration_sec=0.5)
        
        elif self.distance_droite < self.ZONE_VIGILANCE:
            # ✅ Correction douce et progressive
            ratio = 1.0 - (self.distance_droite - self.ZONE_DANGER) / (self.ZONE_VIGILANCE - self.ZONE_DANGER)
            correction = ratio * self.CORRECTION_MAX * 0.4  # 40% de correction max
            self.get_logger().info(f'  Vigilance droite {self.distance_droite:.2f}m', throttle_duration_sec=1.0)
        
        # --- Côté GAUCHE (même logique inversée) ---
        if self.distance_gauche < self.ZONE_CRITIQUE:
            correction = -self.CORRECTION_MAX
            facteur_vitesse = min(facteur_vitesse, 0.3)
            self.get_logger().warn(f' CRITIQUE GAUCHE {self.distance_gauche:.2f}m', throttle_duration_sec=0.3)
        
        elif self.distance_gauche < self.ZONE_DANGER:
            ratio = 1.0 - (self.distance_gauche - self.ZONE_CRITIQUE) / (self.ZONE_DANGER - self.ZONE_CRITIQUE)
            correction = -ratio * self.CORRECTION_MAX * 0.8
            facteur_vitesse = min(facteur_vitesse, 0.6)
            self.get_logger().info(f'  DANGER GAUCHE {self.distance_gauche:.2f}m', throttle_duration_sec=0.5)
        
        elif self.distance_gauche < self.ZONE_VIGILANCE:
            ratio = 1.0 - (self.distance_gauche - self.ZONE_DANGER) / (self.ZONE_VIGILANCE - self.ZONE_DANGER)
            correction = -ratio * self.CORRECTION_MAX * 0.4
            self.get_logger().info(f'  Vigilance gauche {self.distance_gauche:.2f}m', throttle_duration_sec=1.0)
        
        # Saturation finale (sécurité)
        correction = np.clip(correction, -self.CORRECTION_MAX, self.CORRECTION_MAX)
        
        return correction, facteur_vitesse

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)
        
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        #  COMMANDE VISION (direction principale)
        cmd_vision_angular = 0.0
        cmd_vision_linear = self.vitesse_cible

        # === MODE VIRAGE (contrôle vision pur - commande balise active) ===
        if self.en_virage:
            temps_ecoule = now - self.debut_virage
            if temps_ecoule < 3.6:
                cmd_vision_angular = -0.5
                cmd_vision_linear = 0.0  # Arrêt pendant virage
                self.get_logger().info(f'🔄 Virage... {3.6 - temps_ecoule:.1f}s', throttle_duration_sec=0.5)
            else:
                self.en_virage = False
                self.get_logger().info('✅ Virage terminé')
        
        # === MODE NORMAL (vision + détection balises + centrage) ===
        else:
            if tags:
                tag = tags[0]
                tag_id = tag.tag_id
                distance = self.calculer_distance(tag)
                
                # NOUVEAU : Calculer offset de la balise
                offset = self.calculer_offset_horizontal(tag, w)
                
                #  Dessiner ligne centrale de référence
                #  cv2.line(img, (w//2, 0), (w//2, h), (255, 0, 0), 2)
                
                # Dessiner la balise
                for idx in range(len(tag.corners)):
                    cv2.line(img, 
                            tuple(tag.corners[idx-1, :].astype(int)), 
                            tuple(tag.corners[idx, :].astype(int)), 
                            (0, 255, 0), 3)
                
                # Dessiner centre de la balise
                center_x = int(np.mean(tag.corners[:, 0]))
                center_y = int(np.mean(tag.corners[:, 1]))
                cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Affichage infos
                cv2.putText(img, f'ID:{tag_id} {distance*100:.0f}cm', (10, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
                cv2.putText(img, f'Offset: {offset:.0f}px', (10, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                #  CENTRAGE DE LA BALISE (si pas en train d'exécuter commande)
                if abs(offset) > self.SEUIL_CENTRE:
                    #  Correction proportionnelle à l'offset
                    cmd_vision_angular = -offset * self.GAIN_CENTRAGE
                    
                    # Saturation
                    cmd_vision_angular = np.clip(cmd_vision_angular, -0.3, 0.3)
                    
                    self.get_logger().info(f'🎯 Centrage balise (offset: {offset:.0f}px)', throttle_duration_sec=0.5)
                else:
                    # Balise centrée
                    cmd_vision_angular = 0.0
                
                #  RALENTISSEMENT EN APPROCHE (pour meilleure précision)
            if self.vitesse_cible > 0:
                if distance < 1.0:  # Moins de 1 mètre
                    cmd_vision_linear = self.vitesse_cible * 0.6  # 60% vitesse
                    self.get_logger().info(f' Ralentissement approche', throttle_duration_sec=1.0)
                elif distance < 0.7:
                    cmd_vision_linear = self.vitesse_cible * 0.4  # 40% vitesse
                
                #  EXÉCUTION COMMANDES BALISES (à distance d'arrêt)
                if tag_id not in self.balises_vues and distance <= self.distance_arret:
                    
                    self.balises_vues.append(tag_id)
                    
                    if tag_id == 5:  # Balise 5 : Tourner à droite
                        self.get_logger().info(f'🎯 Balise {tag_id} → VIRAGE DROITE')
                        self.en_virage = True
                        self.debut_virage = now
                    
                    elif tag_id == 6:  # Balise 6 : Accélérer
                        self.get_logger().info(f'⚡ Balise {tag_id} → ACCÉLÉRATION')
                        self.vitesse_cible = 0.5
                        cmd_vision_linear = self.vitesse_cible
                    
                    elif tag_id == 7:  # Balise 7 : Tourner à droite
                        self.get_logger().info(f'🎯 Balise {tag_id} → VIRAGE DROITE')
                        self.en_virage = True
                        self.debut_virage = now
                    
                    elif tag_id == 8:  # Balise 8 : Tourner à droite
                        self.get_logger().info(f'🎯 Balise {tag_id} → VIRAGE DROITE')
                        self.en_virage = True
                        self.debut_virage = now
                    
                    elif tag_id == 9:  # Balise 9 : STOP
                        self.get_logger().info(f' Balise {tag_id} → STOP FINAL')
                        self.get_logger().info('=' * 50)
                        self.get_logger().info(' MISSION TERMINÉE')
                        self.get_logger().info(f' Balises détectées : {len(self.balises_vues)}')
                        self.get_logger().info(f'  IDs : {self.balises_vues}')
                        self.get_logger().info('=' * 50)
                        self.vitesse_cible = 0.0
                        cmd_vision_linear = 0.0
            
            # Pas de balise détectée : avancer normalement
            else:
                cmd_vision_linear = self.vitesse_cible
                cmd_vision_angular = 0.0
        
        #  CALCUL CORRECTION LIDAR (sécurité additive)
        correction_lidar, facteur_vitesse = self.calculer_correction_lidar_progressive()
        
        # FUSION VISION + LIDAR (le LiDAR ajoute une contrainte)
        cmd.angular.z = cmd_vision_angular + correction_lidar
        cmd.linear.x = cmd_vision_linear * facteur_vitesse
        
        #  Garantir vitesse minimale (sauf si arrêt voulu)
        if cmd_vision_linear > 0 and cmd.linear.x < self.VITESSE_MIN:
            cmd.linear.x = self.VITESSE_MIN
        
        # Affichage debug
      #  cv2.putText(img, f'G:{self.distance_gauche:.2f}m D:{self.distance_droite:.2f}m A:{self.distance_avant:.2f}m', 
       #            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        #cv2.putText(img, f'Vitesse: {cmd.linear.x:.2f} m/s', 
         #          (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.pub_vel.publish(cmd)
        cv2.imshow("Vision", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = NavSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.pub_vel.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()