# Obstacle Avoidance Node — Documentação do Projeto

## Visão Geral

Este projeto implementa um nó ROS2 de **desvio reativo de obstáculos** para o robô Scout Mini em um ambiente de labirinto (maze) simulado no Gazebo. O robô navega de forma autônoma desde a posição inicial até o objetivo, detectando obstáculos em tempo real com o sensor LaserScan e decidindo a direção de giro mais segura.

| Item         | Valor                              |
|--------------|------------------------------------|
| Pacote       | `scout_control`                    |
| Nó           | `obstacle_avoidance`               |
| Linguagem    | Python 3 / ROS2 Jazzy              |
| Arquivo      | `scout_control/obstacle_avoidance.py` |
| Posição inicial | (-8, 8) — quadrado azul        |
| Objetivo     | (8, -7) — quadrado verde           |

---

## Ambiente de Simulação

- **Simulador**: Gazebo (Harmonic)
- **World**: `maze_class5.sdf`
- **Robô**: Scout Mini
- **Launch**: `scout_sim_maze.launch.py`

O labirinto contém obstáculos (blocos azuis) distribuídos pelo ambiente. O caminho esperado é indicado pelas setas cinzas na imagem do mundo.

---

## Arquitetura do Nó

### Tópicos ROS2

| Tipo        | Tópico       | Mensagem                        | Descrição                        |
|-------------|--------------|----------------------------------|----------------------------------|
| Subscriber  | `/scan`      | `sensor_msgs/LaserScan`         | Dados do sensor LiDAR            |
| Subscriber  | `/odom`      | `nav_msgs/Odometry`             | Posição e orientação (yaw)       |
| Publisher   | `/cmd_vel`   | `geometry_msgs/Twist`           | Comandos de velocidade           |

### Timers

| Timer           | Frequência | Função            |
|-----------------|------------|-------------------|
| `control_loop`  | 10 Hz      | Controle de estado|
| `log_status`    | 1 Hz       | Log de diagnóstico|

---

## Parâmetros Configuráveis

```python
self.linear_speed = 0.3          # Velocidade linear (m/s)
self.angular_speed = 1.5         # Velocidade angular (rad/s)
self.safe_distance = 1.0         # Distância frontal para parar (m)
self.turn_timeout = 160          # Duração do giro em ciclos (16s @ 10Hz)
self.post_turn_duration = 2      # Ciclos de avanço pós-giro (0.2s @ 10Hz)
self.goal_threshold = 1.0        # Raio de chegada ao objetivo (m)
self.final_approach_distance = 2.0  # Distância para ativar abordagem final (m)
```

---

## Setores de Detecção LiDAR

O sensor LiDAR é dividido em três setores angulares:

```
         0° (frente)
             │
     ±10°────┼────±10°    ← Setor FRONTAL (detecta obstáculos)
             │
75°──────────┼──────────-75°
 ↑                         ↑
Setor       Robô       Setor
ESQUERDA              DIREITA
(75°–90°)         (-90°–-75°)
```

| Setor     | Ângulos       | Finalidade                                    |
|-----------|---------------|-----------------------------------------------|
| Frontal   | ±10° (±π/18)  | Detectar obstáculos à frente para parar       |
| Esquerda  | 75° a 90°     | Medir espaço livre à esquerda para decisão   |
| Direita   | -90° a -75°   | Medir espaço livre à direita para decisão    |

> **Por que setores estreitos laterais?**
> Focando nos ângulos mais laterais (75°–90°), evitamos que paredes frontais ou diagonais interfiram na leitura dos lados, garantindo que a decisão de giro reflita melhor o espaço disponível.

---

## Máquina de Estados

O robô opera em 7 estados:

```
         ┌─────────────────────────────────────────────┐
         │                                             │
    ┌────▼─────┐    obstáculo    ┌──────────┐         │
    │ FORWARD  ├────────────────►│   STOP   │         │
    │  (anda)  │                 │ (decide) │         │
    └────┬─────┘                 └────┬─────┘         │
         │                           │ decide         │
         │ perto do goal             ▼                │
         ▼                      ┌──────────┐          │
    ┌────────────┐               │   TURN   │          │
    │ FINAL_TURN │               │ (gira    │          │
    │ (gira dir) │               │ 16 seg)  │          │
    └─────┬──────┘               └────┬─────┘          │
          │                          │ concluído       │
          ▼                          ▼                 │
    ┌────────────┐         ┌──────────────────┐        │
    │ FINAL_MOVE │         │ POST_TURN_FORWARD│        │
    │ (gira 16s) │         │ (anda 0.2s)      │        │
    └─────┬──────┘         └────────┬─────────┘        │
          │                         │                  │
          ▼                         └──────────────────┘
    ┌──────────┐
    │   GOAL   │  ── distância ≤ 1m ──► PARAR
    │ (navega) │
    └──────────┘
```

### Descrição dos Estados

| Estado              | Descrição                                                                 |
|---------------------|---------------------------------------------------------------------------|
| `FORWARD`           | Avança em linha reta a 0.3 m/s. Para quando frente ≤ 1.0 m               |
| `STOP`              | Para completamente. Lê distâncias laterais e decide para qual lado girar  |
| `TURN`              | Gira no próprio eixo por 16 segundos (160 ciclos × 0.1s) na velocidade angular de 1.5 rad/s |
| `POST_TURN_FORWARD` | Avança 2 ciclos (0.2s) na nova direção após girar, para estabilizar       |
| `FINAL_TURN`        | Ativado quando a 2m do objetivo. Para e prepara giro de 90° à direita     |
| `FINAL_MOVE`        | Executa o giro final à direita por 16 segundos                            |
| `GOAL`              | Navega diretamente ao objetivo com correção de heading. Para ao chegar    |

---

## Lógica de Decisão de Giro

No estado `STOP`, o robô compara as distâncias mínimas nos setores laterais:

```python
if self.left_distance > self.right_distance:
    # Vira à esquerda (mais espaço livre)
    self.turn_direction = 1
else:
    # Vira à direita (mais espaço livre)
    self.turn_direction = -1
```

O robô sempre vira para o lado onde a **menor distância até um obstáculo** é **maior**, ou seja, o lado mais livre.

---

## Lógica de Giro por Tempo Fixo

Em vez de usar a odometria para medir o ângulo girado (o que pode falhar por drift), o giro é controlado por **tempo fixo de 16 segundos**:

```python
# Velocidade angular = 1.5 rad/s por 16 segundos
# Ângulo girado ≈ 1.5 × 16 = 24 rad ≈ múltiplos de 90° (na prática ≈ 90° com inércia)
self.turn_timeout = 160  # ciclos a 10Hz = 16 segundos
```

> Nota: Na prática o tempo real efetivo de giro é calibrado empiricamente para que o robô complete aproximadamente 90 graus. Ajuste `turn_timeout` conforme necessário.

---

## Abordagem Final ao Objetivo

Quando a distância euclidiana ao objetivo ≤ 2.0 m:
1. Estado muda para `FINAL_TURN` → gira 90° à direita por 16 segundos
2. Estado muda para `GOAL` → navega diretamente ao objetivo com `atan2`

```python
goal_angle = math.atan2(goal_y - current_y, goal_x - current_x)
yaw_error = normalize_angle(goal_angle - current_yaw)
twist.angular.z = 0.5 * yaw_error  # Correção proporcional de heading
```

---

## Logs de Diagnóstico

O nó emite logs a cada segundo com o estado atual do robô:

```
[INFO] State: FORWARD, Pos: (2.31, -0.00), Yaw: 0.00, Front: 4.68, Left: 1.98, Right: 2.99
[INFO] Obstacle detected, stopping
[INFO] Left distance: 1.99, Right distance: 4.16
[INFO] Turning right
[INFO] Girou noventa graus (5 seconds)
[INFO] Post-turn forward completed, resuming cycle
[INFO] Goal reached!
```

---

## Como Compilar e Executar

### 1. Compilar o pacote

```bash
cd ~/scout_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scout_control
source install/setup.bash
```

### 2. Iniciar a simulação (Terminal 1)

```bash
cd ~/scout_ws
source /opt/ros/jazzy/setup.bash
ros2 launch scout_sim scout_sim_maze.launch.py
```

### 3. Executar o nó de controle (Terminal 2)

```bash
cd ~/scout_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run scout_control obstacle_avoidance
```

---

## Dependências

| Dependência            | Versão / Pacote         |
|------------------------|-------------------------|
| ROS2                   | Jazzy                   |
| Python                 | 3.x                     |
| numpy                  | Standard                |
| sensor_msgs            | LaserScan               |
| nav_msgs               | Odometry                |
| geometry_msgs          | Twist                   |
| scout_sim              | Simulação Gazebo        |
| ros_gz_bridge          | Bridge ROS2 ↔ Gazebo    |

---

## Estrutura de Arquivos

```
scout_ws/
└── src/
    └── scout_mini_ros2/
        └── scout_control/
            ├── scout_control/
            │   └── obstacle_avoidance.py   ← Nó principal
            ├── launch/
            │   └── obstacle_avoidance.launch.py
            ├── setup.py                    ← Entry points
            └── OBSTACLE_AVOIDANCE.md       ← Este documento
```

---

## Histórico de Desenvolvimento e Decisões de Design

| Decisão                          | Motivo                                                             |
|----------------------------------|--------------------------------------------------------------------|
| Setor frontal estreito (±10°)    | Evitar que paredes laterais causem paradas desnecessárias          |
| Setores laterais 75°–90°         | Focar nas leituras mais puramente laterais, sem interferência frontal |
| Giro por tempo fixo (16s)        | Evitar problemas com drift de odometria em giros longos            |
| POST_TURN_FORWARD                | Garantir que o robô avance na nova direção antes de verificar obstáculos novamente |
| `safe_distance = 1.0 m`         | Balancear antecipação de obstáculos com navegação em corredores estreitos |
| Velocidade linear 0.3 m/s        | Velocidade moderada para dar tempo de reação ao sensor             |

---

## Limitações Conhecidas

- O giro por tempo fixo pode não ser exatamente 90° em todos os cenários (depende da inércia do robô)
- A abordagem final assume que basta girar à direita para alinhar com o objetivo — pode falhar se o robô chegar pela direita
- Sem mapeamento: o robô não memoriza o caminho percorrido, podendo revisitar obstáculos
- Sem recuperação: se o robô ficar preso em um canto, não há estado de recuperação

---

## Possíveis Melhorias Futuras

- Adicionar estado `RECOVER` para situações de bloqueio
- Usar odometria para confirmar o ângulo girado como validação secundária
- Implementar SLAM básico para evitar revisitar caminhos
- Ajustar dinamicamente `safe_distance` de acordo com a velocidade atual
