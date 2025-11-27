## husky_navigation

### Pacote ROS 2 para navegação do Husky (Demos Nav2, máquina de estados de waypoints).

## Pré-requisitos e Instalação

Para que este pacote funcione, o ambiente de simulação do Clearpath para ROS 2 Jazzy deve estar corretamente instalado e configurado.

Configuração do Computador (Offboard PC):

Siga o guia oficial para instalar o ROS 2 Jazzy e as ferramentas essenciais.

### Link: https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc/

Instalação do Simulador Clearpath:

Siga o guia oficial para instalar o Gazebo e o meta-pacote ros-jazzy-clearpath-simulator. Recomenda-se a instalação via Source Install (Item 4) se você planeja adicionar mundos customizados.

### Link: https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install/

## Configuração do Robô (robot.yaml):

Certifique-se de que seu arquivo $HOME/clearpath/robot.yaml (ou /etc/clearpath/robot.yaml) está configurado para o robô desejado (ex: a200_0000).

Build (Compilação)

Este pacote deve ser compilado dentro do mesmo workspace onde o simulador (via Source Install) foi instalado (ex: clearpath_ws).

### Navegue até a raiz do workspace
cd ~/clearpath_ws

### Compile o workspace (incluindo este pacote)
colcon build --symlink-install


## Como Usar (Workflows)

Existem dois modos principais de operação: Mapeamento (para criar mapas) e Navegação (para seguir rotas).

### Workflow A: Mapeamento (Gerar um novo mapa com SLAM)

Use este workflow para mapear um novo ambiente.

1. Lance a Simulação (Gazebo):
(Em um novo terminal)

### Lance o Gazebo com RViz (substitua 'warehouse' pelo seu mundo)
ros2 launch clearpath_gz simulation.launch.py world:=warehouse rviz:=true


2. Lance o Stack de Mapeamento (SLAM + Nav2 + RViz):
(Em um novo terminal)
Este comando (baseado no arquivo clearpath_full_slam.launch.py) inicia o slam_toolbox, o nav2 (em modo SLAM) e o rviz configurado.

ros2 launch husky_navigation clearpath_full_slam.launch.py


3. Mova o Robô:
Use o joystick ou o RViz (botão "Teleop" no painel do clearpath_viz) para dirigir o robô e construir o mapa.

Lembre de checar o tópico no gazebo, exemplo: teleop do Gazebo setado no /cmd_vel, mas a simulação (e o Nav2) espera no /a200_0000/cmd_vel. O teleop do RViz geralmente usa o namespace correto.

4. Salve o Mapa:
(Em um novo terminal)
Quando estiver satisfeito com o mapa, use o map_saver_cli para salvá-lo.

ros2 run nav2_map_server map_saver_cli --ros-args -p map_subscribe_transient_local:=true -r __ns:=/a200_0000 -- -f "meu_mapa"

Isso salvará meu_mapa.yaml e meu_mapa.pgm no seu diretório atual.

Adicionando Novos Mapas ao Pacote

O pacote já inclui uma pasta maps (configurada no CMakeLists.txt) com um mapa de exemplo (office_map.yaml). Se você criar um novo mapa (como meu_mapa.yaml no Workflow A), siga estes passos para adicioná-lo:

Mova seus arquivos de mapa:

mv meu_mapa.yaml meu_mapa.pgm ~/clearpath_ws/src/husky_navigation/maps/


Recompile:

cd ~/clearpath_ws
colcon build --packages-select husky_navigation


### Workflow B: Navegação (Usar um mapa existente)

Use este workflow para carregar um mapa pré-construído e navegar com o waypoint_state_machine.

1. Lance a Simulação (Gazebo):
(Em um novo terminal)

### Lance o Gazebo
ros2 launch clearpath_gz simulation.launch.py world:=warehouse rviz:=true


2. Lance o Stack de Navegação:
(Em um novo terminal)
Este launch file (nav2_husky.launch.py) agora aceita um argumento map para carregar mapas customizados.

### --- OPÇÃO 1 (Padrão): Usar o mapa 'warehouse.yaml' (dos demos) ---
ros2 launch husky_navigation nav2_husky.launch.py

### --- OPÇÃO 2 (Exemplo): Usar o mapa 'office_map.yaml' (deste pacote) ---
ros2 launch husky_navigation nav2_husky.launch.py map:='$(find-pkg-share husky_navigation)/maps/office_map.yaml'

### --- OPÇÃO 3 (Customizado): Usar o seu 'meu_mapa.yaml' ---
(Assumindo que você seguiu os passos da seção "Adicionando Novos Mapas ao Pacote")
ros2 launch husky_navigation nav2_husky.launch.py map:='$(find-pkg-share husky_navigation)/maps/meu_mapa.yaml'


3. Execute a Máquina de Estados (State Machine):
(Em um novo terminal)
Com a navegação ativa, execute o script de waypoints.

ros2 run husky_navigation waypoint_state_machine.py


### Workflow C: Navegação por Coordenadas GPS

Use este workflow para navegar o robô usando coordenadas GPS reais convertidas para o mapa local.

#### Passo 1: Calibração GPS (Primeira Execução)

Antes de usar a navegação por GPS, você precisa calibrar a conversão entre coordenadas GPS e coordenadas do mapa local. Este processo associa um ponto GPS conhecido a uma posição no mapa.

1. Lance a Simulação e o Stack de Navegação (conforme Workflow B):

```bash
# Terminal 1: Gazebo
ros2 launch clearpath_gz simulation.launch.py world:=warehouse rviz:=true

# Terminal 2: Nav2
ros2 launch husky_navigation nav2_husky.launch.py
```

2. Execute o script de calibração (em um novo terminal):

```bash
ros2 run husky_navigation gps_calibration.py
```

3. Em outro terminal, monitore a posição do GPS do robô:

```bash
ros2 topic echo /a200_0000/sensors/gps_0/fix
```

4. Posicione o robô em um local conhecido e anote:
   - **Posição no mapa (do script de calibração)**: ex. `x=-1.93, y=1.11`
   - **Coordenadas GPS**: ex. `latitude: -30.000012, longitude: -51.000011`

5. Esses valores serão usados como referência na navegação GPS.

#### Passo 2: Configurar Waypoints GPS

Edite o arquivo `config/gps_waypoints.yaml` com seus waypoints:

```yaml
waypoints:
  gps_points:
    - latitude: -30.000010   # Primeira coordenada
      longitude: -51.000010
      yaw: 0.0  # Orientação em radianos (opcional)
    
    - latitude: -30.000015   # Segunda coordenada
      longitude: -51.000010
      yaw: 1.57
    
    - latitude: -30.000015   # Terceira coordenada
      longitude: -51.000020
      yaw: 0.0

navigation:
  timeout: 300.0
  waypoint_tolerance: 0.5
```

**Dicas:**
- Diferença de 0.00001° ≈ 1 metro no mapa
- Use o valor de `yaw` para controlar a orientação do robô (em radianos):
  - `0.0` = virado para leste
  - `1.57` ≈ π/2 = virado para norte
  - `3.14` ≈ π = virado para oeste

#### Passo 3: Executar Navegação GPS

Com o Nav2 em execução, lance o script de navegação GPS (em um novo terminal):

```bash
# Usar valores de referência padrão (-30.0, -51.0)
ros2 run husky_navigation navigate_gps_waypoints.py config/gps_waypoints.yaml

# OU especificar coordenadas de referência calibradas
ros2 run husky_navigation navigate_gps_waypoints.py config/gps_waypoints.yaml \
  --ref-lat -30.000012 --ref-lon -51.000011
```

O script:
1. Carregará os waypoints GPS do arquivo YAML
2. Converterá as coordenadas GPS para posições no mapa
3. Enviará o robô para navegar através de todos os waypoints em sequência
4. Exibirá feedback em tempo real

#### Parâmetros do Script

```
--ref-lat <latitude>    Latitude de referência (onde mapeia-se ao ponto base)
--ref-lon <longitude>   Longitude de referência
--namespace <ns>        Namespace do robô (padrão: a200_0000)
```

#### Exemplo Prático

```bash
# 1. Calibração (execute uma vez)
ros2 run husky_navigation gps_calibration.py

# 2. Editar waypoints em config/gps_waypoints.yaml

# 3. Navegação
ros2 run husky_navigation navigate_gps_waypoints.py config/gps_waypoints.yaml \
  --ref-lat -30.000012 --ref-lon -51.000011
```

