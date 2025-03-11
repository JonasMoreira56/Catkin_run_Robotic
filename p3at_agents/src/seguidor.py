#!/usr/bin/env python3
# remember to make this file executable (`chmod +x`) before trying to run it

import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sim_p3at/camera/image_raw", 
                                         Image, self.image_callback, 
                                         queue_size=1, buff_size=2**24,
                                         tcp_nodelay=True)
        self.cmd_vel_pub = rospy.Publisher('/sim_p3at/cmd_vel', Twist, queue_size=10)
        
        self._init_yolo()
        
        self.twist = Twist()
        self.last_person_x = None  # Armazena a última posição da pessoa
        self.last_person_y = None

        # Parâmetros para controle de proximidade
        self.min_safe_distance = 0.20  # Proporção da área da imagem (quando a pessoa ocupa 20% da imagem, é muito próxima)
        self.max_follow_distance = 0.05  # Limite para iniciar o seguimento (pessoa ocupando menos de 5% da imagem)
        self.ideal_distance = 0.15    # Distância ideal para manter da pessoa (15% da imagem)
        
    def _init_yolo(self):
        """Inicializa o modelo YOLO apenas uma vez para economizar memória"""
        self.dir = os.path.dirname(__file__)
        
        print("DIRETORIO:", self.dir)
        
        # Carregamento de labels
        labels_path = os.path.join(self.dir, "models/coco.names")
        self.LABELS = open(labels_path).read().strip().split("\n")
        
        # Carregamento do modelo
        weights_path = os.path.join(self.dir, "models/yolov4-tiny.weights")
        config_path = os.path.join(self.dir, "models/yolov4-tiny.cfg")
        
        print("PESOS:", weights_path)
        
        try:
            self.model = cv2.dnn.readNetFromDarknet(config_path, weights_path)
            # Configuração explícita para CPU
            self.model.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
            self.model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            print("Usando CPU para detecção YOLO")
        except Exception as e:
            print(f"Erro ao carregar o modelo: {e}")
            raise
            
        # Camadas de saída
        layer_indexes = self.model.getUnconnectedOutLayers()
        self.layer_names = [self.model.getLayerNames()[i - 1] for i in layer_indexes.flatten()]
        
        print("Init Environment P3AT V2 (Image)")
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        cv_image = cv2.resize(cv_image, (640, 380), interpolation= cv2.INTER_LINEAR)

        self.follow_person(cv_image)
        
        # Exibir imagem da câmera
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def calculate_distance_control(self, area_ratio):
        """
        Calcula o controle de velocidade com base na distância (área relativa da caixa)
        Retorna:
        - Uma flag indicando se é seguro se mover para frente
        - A velocidade linear adequada baseada na distância
        - Um valor de status (1: muito próximo, 0: distância ideal, -1: muito longe)
        """
        # Inicializa as variáveis de retorno
        safe_to_move = True
        linear_speed = 0.0
        distance_status = 0
        
        # 1. Verificação de proximidade crítica - Parar completamente se muito próximo
        if area_ratio >= self.min_safe_distance:
            safe_to_move = False
            linear_speed = 0.0
            distance_status = 1  # Muito próximo
            
        # 2. Muito longe - Velocidade máxima para alcançar
        elif area_ratio <= self.max_follow_distance:
            safe_to_move = True
            linear_speed = 0.6  # Velocidade máxima para alcançar a pessoa
            distance_status = -1  # Muito longe
            
        # 3. Na zona de seguimento - Ajustar velocidade com base na distância
        else:
            safe_to_move = True
            # Calcular velocidade proporcional à diferença entre a distância atual e a ideal
            distance_factor = (self.ideal_distance - area_ratio) / (self.ideal_distance - self.max_follow_distance)
            
            # Limitar o fator entre -1 e 1
            distance_factor = max(-1.0, min(1.0, distance_factor))
            
            # Se estiver mais próximo que o ideal, recuar levemente (velocidade negativa)
            if area_ratio > self.ideal_distance:
                # Quanto mais próximo, mais forte o recuo, mas com um limite máximo
                linear_speed = max(-0.2, -0.3 * (area_ratio - self.ideal_distance) / (self.min_safe_distance - self.ideal_distance))
                distance_status = 0.5  # Próximo, mas não crítico
            # Se estiver mais longe que o ideal, avançar
            else:
                # Mapear para uma velocidade entre 0.1 e 0.5
                linear_speed = 0.1 + 0.4 * distance_factor
                distance_status = -0.5  # Longe, mas não crítico
        
        return safe_to_move, linear_speed, distance_status

    def follow_person(self, frame):
        """Detecta e segue pessoas no frame, com controle de movimento melhorado"""
        height, width, _ = frame.shape
        center_x = width // 2
        center_y = height // 2  # Definir center_y no início da função
        
        # Preparar imagem para o YOLO
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.model.setInput(blob)
        outputs = self.model.forward(self.layer_names)
        
        # Listas para armazenar detecções
        boxes = []
        confidences = []
        class_ids = []
        person_centers = []
        
        # Processar as saídas do YOLO
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if class_id == 0 and confidence > 0.5:  # Classe 0 corresponde a 'pessoa'
                    center_x_detected = int(detection[0] * width)
                    center_y_detected = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x_detected - w / 2)
                    y = int(center_y_detected - h / 2)
                    boxes.append([x, y, w, h])
                    person_centers.append((center_x_detected, center_y_detected))
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        # Inicializar comando de velocidade
        self.twist = Twist()
        
        # Desenhar linha central para referência
        cv2.line(frame, (center_x, 0), (center_x, height), (255, 255, 0), 1)
        
        if len(boxes) > 0:
            # Identificar o índice da maior caixa (pessoa mais próxima)
            largest_box_idx = max(range(len(boxes)), key=lambda i: boxes[i][2] * boxes[i][3])
            x, y, w, h = boxes[largest_box_idx]
            person_center_x, person_center_y = person_centers[largest_box_idx]
            
            # Calcular a área relativa da caixa que representa a pessoa
            box_area = w * h
            area_ratio = box_area / (width * height)
            
            # Aplicar o controle de distância
            safe_to_move, linear_x, distance_status = self.calculate_distance_control(area_ratio)
            
            # Adicionar indicadores visuais de distância
            distance_status_text = ""
            distance_color = (255, 255, 255)  # Cor padrão (branco)
            
            if distance_status == 1:
                distance_status_text = "MUITO PRÓXIMO - PARADO"
                distance_color = (0, 0, 255)  # Vermelho
            elif distance_status == 0.5:
                distance_status_text = "Próximo - Recuando"
                distance_color = (0, 165, 255)  # Laranja
            elif distance_status == 0:
                distance_status_text = "Distância ideal"
                distance_color = (0, 255, 0)  # Verde
            elif distance_status == -0.5:
                distance_status_text = "Seguindo"
                distance_color = (0, 255, 255)  # Amarelo
            elif distance_status == -1:
                distance_status_text = "Muito longe - Acelerando"
                distance_color = (255, 0, 0)  # Azul
            
            # Desenhar a caixa ao redor da pessoa detectada com cor baseada na distância
            cv2.rectangle(frame, (x, y), (x + w, y + h), distance_color, 2)
            cv2.circle(frame, (person_center_x, person_center_y), 5, (0, 0, 255), -1)
            
            # Exibir status de distância
            cv2.putText(frame, distance_status_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, distance_color, 2)
            
            # Exibir área relativa (proporção da caixa)
            cv2.putText(frame, f"Área: {area_ratio:.3f}", (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Calcular o erro de posição da pessoa em relação ao centro da imagem
            error_x = person_center_x - center_x
            
            # Parâmetros de controle ajustáveis
            kp_angular = 0.003  # Ganho proporcional para controle angular
            max_angular_speed = 1.0  # Velocidade angular máxima
            
            # Calcular velocidade angular com base na posição da pessoa
            angular_z = -kp_angular * error_x
            
            # Limitar a velocidade angular
            angular_z = max(min(angular_z, max_angular_speed), -max_angular_speed)
            
            # Detectar movimento da pessoa entre frames
            if hasattr(self, 'last_person_x') and self.last_person_x is not None and hasattr(self, 'last_person_y') and self.last_person_y is not None:
                # Calcular deslocamento da pessoa
                dx = person_center_x - self.last_person_x
                dy = person_center_y - self.last_person_y
                
                # Detectar movimento lateral significativo
                if abs(dx) > abs(dy) and abs(dx) > 20:
                    # Aumentar resposta angular para curvas
                    angular_z *= 1.5
                    
                # Exibir informações de movimento na imagem
                cv2.putText(frame, f"Movimento: dx={dx:.1f}, dy={dy:.1f}", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Atualizar posição anterior da pessoa
            self.last_person_x = person_center_x
            self.last_person_y = person_center_y
            
            # Definir velocidades do robô
            self.twist.linear.x = linear_x
            self.twist.angular.z = angular_z
            
            # Exibir informações de controle na imagem
            cv2.putText(frame, f"Erro: {error_x}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, f"Vel: lin={self.twist.linear.x:.2f}, ang={self.twist.angular.z:.2f}", 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                       
            # Adicionar linha do centro para a pessoa
            cv2.line(frame, (center_x, height//2), (person_center_x, person_center_y), (255, 0, 0), 2)
        else:
            # Se nenhuma pessoa for detectada, parar o robô
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            
            # Resetar variáveis de rastreamento
            self.last_person_x = None
            self.last_person_y = None
            
            # Exibir mensagem na imagem
            cv2.putText(frame, "Nenhuma pessoa detectada", (center_x - 100, center_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Publicar os comandos de velocidade
        self.cmd_vel_pub.publish(self.twist)
        
        return frame  # Retorna o frame com as anotações visuais

        
    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
