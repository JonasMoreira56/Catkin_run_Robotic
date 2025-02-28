# Projeto ROS para Controle de Robô com Câmera e Agente de Aprendizado por Reforço

Este projeto ROS (Robot Operating System) consiste em três pacotes principais: `axis_camera`, `pionner_control` e `rl_agent`. Cada pacote tem uma funcionalidade específica para controlar um robô, capturar imagens de uma câmera e aplicar um agente de aprendizado por reforço.

## Estrutura do Projeto

## Pacotes

### axis_camera

Este pacote é responsável por capturar imagens de uma câmera IP e publicá-las no tópico `/camera/image_raw`.

- **Arquivo de Nó:** [camera_node.py](axis_camera/src/camera_node.py)
- **Lançamento:** [camera.launch](axis_camera/launch/camera.launch)

### pionner_control

Este pacote controla o movimento do robô publicando comandos de velocidade no tópico `/cmd_vel`.

- **Arquivo de Nó:** [control_node.py](pionner_control/src/control_node.py)
- **Lançamento:** [control.launch](pionner_control/launch/control.launch)

### rl_agent

Este pacote implementa um agente de aprendizado por reforço que processa imagens da câmera e decide ações para controlar o robô.

- **Arquivo de Nó:** [agent_node.py](rl_agent/src/agent_node.py)
- **Lançamento:** [rl_agent.launch](rl_agent/launch/rl_agent.launch)

## Como Executar

1. **Configurar o Ambiente ROS:**
   Certifique-se de que o ROS está instalado e configurado corretamente no seu sistema.

2. **Compilar o Workspace:**
   Navegue até o diretório do workspace e compile os pacotes:
   ```sh
   cd /home/jonas/catkin_ws
   catkin_make

3. **Fonte o Setup do Workspace:**
   ```sh
   source devel/setup.bash

4. **Executar os Nós**
   - Inicie o nó da câmera:
   ```sh
   roslaunch axis_camera camera.launch
  - Inicie o nó de controle
    ```sh
    roslaunch pionner_control control.launch
 - Inicie o nó do agente de aprendizado por reforço:
    ```sh
    roslaunch rl_agent rl_agent.launch

## Dependências
- ROS (Kinetic ou superior)
- OpenCV
- NumPy
- Requests
- PyTorch
- cv_bridge
- sensor_msgs
- geometry_msgs

## Licença
Este projeto está licenciado sob a licença [TODO].

## Manutenção
Este projeto é mantido pelo Pesquisador Jonas Moreira - Projeto Super IOT-WP3. Para dúvidas ou sugestões, entre em contato pelo email jonas.barbosa@ufam.edu.br

