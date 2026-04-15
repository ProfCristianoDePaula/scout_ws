# Challenge 2 – Scout Mini: Navegação Autônoma com Desvio de Obstáculos

**Disciplina:** AMR – Autonomous Mobile Robots — UFSCar 2026  
**Robô:** Scout Mini  
**Arquivo principal:** `src/scout_mini_ros2/scout_control/scout_control/challenge2_navigator.py`

---

## 1. Objetivo

Navegar autonomamente em um corredor com 3 caixas (obstáculos) posicionadas aleatoriamente, partindo de um ponto inicial até o final do corredor, sem colisões.

---

## 2. Arquitetura do Nó

Toda a lógica está encapsulada na classe `ObstacleAvoidanceNode` (herda de `rclpy.node.Node`). Nenhuma lógica é implementada em `main()`.

### Tópicos

| Direção | Tópico | Tipo | Descrição |
|---------|--------|------|-----------|
| Subscribe | `/scan` | `sensor_msgs/LaserScan` | Dados do LiDAR 360° |
| Subscribe | `/odom` | `nav_msgs/Odometry` | Posição e orientação do robô |
| Publish | `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidade |

---

## 3. Estratégia de Controle — Reativo Puro

A abordagem final utiliza **controle reativo puro**: a cada leitura do LiDAR, o robô decide **sem memória de estado** o que fazer. Isso evita o problema de máquinas de estado que travavam o robô em loops de giro.

### Setores do LiDAR

O scan é dividido em 3 setores a partir do índice central (frente do robô):

| Setor | Ângulo | Índices |
|-------|--------|---------|
| Frente | ±15° | `center-15` a `center+15` |
| Esquerda | +15° a +60° | `center+15` a `center+60` |
| Direita | -60° a -15° | `center-60` a `center-15` |

O cone frontal estreito (±15°) garante que o robô só reage a obstáculos realmente na sua rota, e não às paredes laterais.

### Comportamentos (prioridade de cima para baixo)

| Estado | Condição | Vel. Linear | Vel. Angular | Descrição |
|--------|----------|-------------|--------------|-----------|
| **RÉ** | CRITICO por >8 ciclos | -0.20 m/s | ±0.8 rad/s | Marcha ré + gira para se destravar |
| **CRITICO** | `frente < 0.70m` ou `lado < 0.50m` | 0.0 m/s | ±1.0 rad/s | Para e gira forte para o lado livre |
| **DESVIO** | `frente < 1.50m` | 0.10 m/s | ±0.8 rad/s | Avança devagar + desvia para o lado livre |
| **LIVRE** | `frente ≥ 1.50m` | 0.30 m/s | proporcional | Cruzeiro com correção de heading ao goal |

### Correção de Parede (sempre ativa)

Se `esquerda < 0.60m` → adiciona -0.5 rad/s (afasta da parede esquerda)  
Se `direita < 0.60m` → adiciona +0.5 rad/s (afasta da parede direita)

### Correção de Heading ao Goal

No modo LIVRE, o robô calcula o ângulo até o goal `(13.2, 0.0)` e aplica correção proporcional:

```
angular = clamp(1.20 × heading_error, -0.8, 0.8)
```

---

## 4. Parâmetros de Distância de Segurança

| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `FRONT_STOP` | 0.70 m | Distância frontal crítica — para e gira |
| `FRONT_SLOW` | 1.50 m | Distância frontal de alerta — desacelera e desvia |
| `SIDE_DANGER` | 0.50 m | Distância lateral perigosa — ativa modo CRITICO |
| `WALL_MIN` | 0.60 m | Distância mínima das paredes — correção angular |
| `GOAL_TOL` | 0.60 m | Tolerância para considerar goal atingido |

Esses valores foram calibrados empiricamente para o corredor do laboratório, considerando a largura do Scout Mini e o tamanho das caixas.

---

## 5. Recuperação de Encurralamento (Marcha Ré)

Quando o robô fica preso (frente, esquerda e direita bloqueados), um contador `_stuck_count` incrementa a cada ciclo em CRITICO. Após **8 ciclos consecutivos** (~4 segundos), o robô entra em modo RÉ:

- Velocidade linear: **-0.20 m/s** (para trás)
- Velocidade angular: **±0.8 rad/s** (gira para o lado mais livre)

Ao ganhar espaço, o contador zera e o controle reativo normal retoma.

---

## 6. Detecção e Estimativa de Pose das Caixas

### Método

1. Converte cada ponto do LiDAR em coordenadas cartesianas (x, y) no frame do sensor
2. Aplica **clusterização euclidiana**: pontos consecutivos com distância < 0.25m pertencem ao mesmo cluster
3. Filtra clusters com ≥ 5 pontos (elimina ruído)
4. Calcula o centroide de cada cluster como posição estimada da caixa

### Relatório

Ao atingir o goal, o nó imprime as posições das caixas detectadas tanto no frame do sensor quanto no frame odométrico (transformação usando posição e yaw atuais do robô).

---

## 7. Como Executar

### Pré-requisitos

- ROS 2 Jazzy instalado
- Workspace compilado

### Compilar

```bash
cd ~/scout_ws
colcon build --packages-select scout_control
source install/setup.bash
```

### Executar

Terminal 1 — Simulação (Gazebo + Scout Mini):
```bash
# Lançar a simulação do Scout Mini no corredor
ros2 launch scout_sim scout_corridor.launch.py
```

Terminal 2 — Navegação autônoma:
```bash
cd ~/scout_ws
source install/setup.bash
ros2 run scout_control challenge2_navigator
```

### Verificar funcionamento

O nó imprime logs em tempo real:
- `LIVRE` → navegando normalmente ao goal
- `DESVIO` → detectou obstáculo, desviando
- `CRITICO` → muito perto, parado e girando
- `RE` → encurralado, dando marcha ré
- `OBJETIVO ATINGIDO` → chegou ao final do corredor
- `RELATORIO DE CAIXAS` → posições estimadas das caixas

---

## 8. Evolução do Desenvolvimento

1. **Versão inicial (máquina de estados):** O robô entrava em estado EVITANDO e ficava preso girando até virar 180° porque a condição de saída (`front > threshold`) nunca era satisfeita no corredor estreito (o robô girava e via a parede).

2. **Ajuste de velocidades:** Redução da velocidade angular e aumento da linear durante desvio para fazer arcos em vez de giros no lugar. Melhorou mas ainda ficava preso.

3. **Controle reativo puro (versão final):** Eliminação da máquina de estados. A cada ciclo o robô decide independentemente: se a frente está livre, vai ao goal; se não, desvia. Quando a caixa sai do cone frontal estreito (±15°), ele imediatamente retoma rumo ao objetivo.

4. **Aumento das distâncias de segurança:** `FRONT_SLOW` de 1.0m → 1.5m e `FRONT_STOP` de 0.5m → 0.7m para o robô começar a desviar mais cedo e não trombar nas caixas.

5. **Recuperação de encurralamento:** Adição de marcha ré automática quando o robô fica cercado por mais de 8 ciclos, permitindo se destravar de cantos apertados.
