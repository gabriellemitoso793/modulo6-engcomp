# Turtlebot Teleoperado

## Objetivo

O objetivo deste projeto é fazer o setup e interagir com o turtlebot, compreendendo os conceitos básicos para uso do ROS em rede e dos pacotes para interação com o robô.

## Descrição

Este projeto consiste em desenvolver um script que seja capaz de fazer a leitura de teclas pressionadas pelo usuário e, utilizando um publisher no tópico adequado, provocar a movimentação do robô. Para isso, foram desenvolvidos dois componentes principais:

1. Uma interface de usuário (feita em terminal). Essa interface é capaz de detectar em tempo real os botões pressionados pelo usuário e dar um feedback da velocidade do robô em tempo real.
2. Um nó de ROS 2 capaz de comandar o robô utilizando o tópico adequado e que seja capaz de verificar se o robô está inicializado e disponível para receber suas mensagens antes de enviá-las.

## Estrutura de pastas 

meu_workspace/: Contém os arquivos ROS 2 para controle do robô.

ponderada-semana3/: Contém os arquivos relacionados à simulação do Turtlebot no Webots.

## Instalação e Configuração

1. **Clone o Repositório:**
Primeiro é necesário clonar o repositório, para isso, acesse o terminal do seu computador e execute os seguintes comandos:

   ```bash
   git clone https://github.com/gabriellemitoso793/modulo6-engcomp.git
   cd modulo6-engcomp
   ```
2. **Construa o Projeto:**
Acesse a pasta 'meu_wokspace' no seu terminal e execute os seguintes comandos para construir o projeto e acionar a CLI:

   ```bash
    cd modulo6-engcomp/meu_workspace
    colcon build
    source install/local_setup.bash
    ros2 run turtle_gabi turtle_gabi
   ```

3. **Abra um novo terminal**
Neste terminal, navegue até a pasta 'ponderada-semana3' e execute os seguintes comandos para abrir o reboots:
    ```bash
    cd modulo6-engcomp/ponderada-semana3
    ros2 launch webots_ros2_turtlebot robot_launch.py
    ```

4. **Constrole o robô através do seu teclado**
No terminal 1, será possível controlar o robô através das teclas 'a', 'w', 's' e 'd'.

## Vídeo demonstrativo

Veja em: [Robô teleoperado](https://drive.google.com/file/d/1d7U-fgTrm3wTmkUxU92KTtcg9lvslpaY/view?usp=sharing)
