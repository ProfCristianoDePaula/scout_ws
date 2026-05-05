# AA07 – Scout Mini: Localização Autônoma com AMCL

**Disciplina:** AMR – Autonomous Mobile Robots — UFSCar 2026  
**Robô:** Scout Mini  
**Tema:** Adaptive Monte Carlo Localization (AMCL)  
**Mapa:** `maze_class7_map` (labirinto)  
**Framework:** ROS 2 Jazzy + Nav2

---

## 1. Objetivo

Validar o funcionamento do algoritmo **AMCL (Adaptive Monte Carlo Localization)** para localização do Scout Mini dentro de um labirinto pré-mapeado. A atividade inclui:

1. **Tarefa 1:** Observar a convergência correta da nuvem de partículas com pose inicial conhecida
2. **Tarefa 2:** Testar o comportamento com pose inicial errada e verificar recuperação
3. **Tarefa 3:** Modificar parâmetros do AMCL (ex: `min_particles`) e comparar desempenho

---

## 2. Stack de Navegação

### Componentes Principais

| Componente      | Pacote            | Função                      |
| --------------- | ----------------- | --------------------------- |
| **Nav2**        | `nav2_bringup`    | Stack de navegação autônoma |
| **AMCL**        | `nav2_amcl`       | Localização probabilística  |
| **Controlador** | `nav2_controller` | Seguir caminho planejado    |
| **Planejador**  | `nav2_planner`    | Gerar trajetória até goal   |
| **RViz**        | `rviz2`           | Visualização em tempo real  |

### Fluxo de Execução

```
[Simulação Gazebo] → [odometria + scan LiDAR] → [AMCL] → [Nav2] → [Scout Mini]
                                                   ↓
                                            nuvem de partículas
```

---

## 3. Configuração AMCL

### Arquivo de Parâmetros

Localização: `src/scout_mini_ros2/scout_sim/config/nav2_params.yaml`

#### Parâmetros Críticos

| Parâmetro          | Valor (padrão) | Descrição                                   |
| ------------------ | -------------- | ------------------------------------------- |
| `min_particles`    | 500            | Número mínimo de partículas na nuvem        |
| `max_particles`    | 2000           | Número máximo de partículas                 |
| `initial_pose_x`   | 0.0            | Pose inicial em X (em metros)               |
| `initial_pose_y`   | 0.0            | Pose inicial em Y (em metros)               |
| `initial_pose_a`   | 0.0            | Pose inicial angular em theta (em radianos) |
| `set_initial_pose` | true           | Se deve usar pose inicial informada         |

### Nuvem de Partículas

- **Dispersão inicial:** Partículas distribuídas em volta da pose inicial
- **Convergência:** À medida que o robô se move, partículas inconsistentes com sensores (LiDAR) são descartadas
- **Objetivo:** Concentrar partículas na posição real do robô no mapa

---

## 4. Mapa do Ambiente

- **Arquivo:** `maze_class7_map.pgm` + `maze_class7_map.yaml`
- **Tipo:** Gridmap (255 = livre, 0 = obstáculo)
- **Coordenada de origem:** Ver `maze_class7_map.yaml`
- **Uso:** AMCL localiza o robô dentro deste mapa usando varreduras LiDAR

---

## 5. Como Executar

### Pré-requisitos

- ROS 2 Jazzy instalado
- Workspace compilado
- Mapa `maze_class7_map.pgm` disponível

### Compilar

```bash
cd ~/scout_ws
colcon build --packages-select scout_sim
source install/setup.bash
```

### Executar a Simulação com AMCL

Terminal 1 — Simulação + AMCL + Nav2:

```bash
cd ~/scout_ws
source install/setup.bash
ros2 launch scout_sim scout_sim_maze_nav2.launch.py
```

Terminal 2 — RViz para visualização (opcional, mas recomendado):

```bash
cd ~/scout_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d <path_to_config>/scout_nav2.rviz
```

### Monitorar Parâmetros AMCL em Tempo Real

```bash
# Verificar min_particles atual
ros2 param get /amcl min_particles

# Visualizar nuvem de partículas
ros2 topic echo /particle_cloud

# Status do AMCL
ros2 service call /amcl/get_state std_srvs/srv/Empty
```

---

## 6. Tarefas da Atividade AA07

### Tarefa 1: Observar Convergência Correta

**Objetivo:** Validar que a nuvem de partículas converge corretamente quando a pose inicial é conhecida.

**Procedimento:**

1. Lançar a simulação com pose inicial correta (usar valor padrão ou informado)
2. Observar em RViz a evolução da nuvem de partículas
3. Registrar screenshots em 3 momentos:
   - Início (partículas dispersas)
   - Meio da convergência (~30 segundos)
   - Convergência final (< 30 segundos ou < 5 segundos)
4. Documentar tempo até convergência

**Resultado esperado:** Partículas convergem rapidamente para a posição real do robô.

**Entrega:** Screenshots + análise em `Docs/Tarefa 1 - Observar a convergencia correta/`

---

### Tarefa 2: Teste com Pose Inicial Errada

**Objetivo:** Validar recuperação do AMCL quando inicializado com pose errada.

**Procedimento:**

1. Modificar parâmetro `initial_pose_x`, `initial_pose_y` ou `initial_pose_a` em `nav2_params.yaml` para valor significativamente errado
   - Ex: pose real = (0, 0), pose inicial = (5, 5, π)
2. Recompilar: `colcon build --packages-select scout_sim --symlink-install`
3. Lançar simulação e observar recuperação
4. Registrar screenshots:
   - Estado inicial (partículas espalhadas)
   - Fase de transição
   - Recuperação final
5. Documentar tempo até reconvergência

**Resultado esperado:** Mesmo com pose errada, AMCL converge para a posição correta após alguns ciclos de movimento e sensor update.

**Entrega:** Screenshots + análise em `Docs/Tarefa 2 - Testar Pose Inicial errada/`

---

### Tarefa 3: Modificar Parâmetros (min_particles)

**Objetivo:** Comparar impacto de `min_particles` no desempenho do AMCL.

**Procedimento:**

1. Alterar `min_particles` em `nav2_params.yaml` para valores diferentes:
   - Cenário A: `min_particles = 100`
   - Cenário B: `min_particles = 500` (padrão)
   - Cenário C: `min_particles = 2000`
2. Para cada cenário:
   - Recompilar com `colcon build --packages-select scout_sim --symlink-install`
   - Executar simulação
   - Medir tempo de convergência
   - Validar com `ros2 param get /amcl min_particles`
   - Observar quantidade de partículas em `/particle_cloud`
3. Tabular resultados e análise comparativa

**Métricas a coletar:**

- Tempo até convergência (segundos)
- Precisão final da localização
- Estabilidade da pose (variância)
- Comportamento de `/particle_cloud`

**Entrega:** Tabelas + gráficos + análise em `Docs/Tarefa 3 - Modificar Parametros/`

---

## 7. Tópicos e Serviços Importantes

### Tópicos Publicados

| Tópico            | Tipo                                      | Descrição                               |
| ----------------- | ----------------------------------------- | --------------------------------------- |
| `/particle_cloud` | `geometry_msgs/PoseArray`                 | Nuvem de partículas do AMCL             |
| `/amcl_pose`      | `geometry_msgs/PoseWithCovarianceStamped` | Posição estimada (média das partículas) |
| `/map`            | `nav_msgs/OccupancyGrid`                  | Mapa estático do labirinto              |

### Tópicos Subscritos

| Tópico  | Tipo                    | Descrição                   |
| ------- | ----------------------- | --------------------------- |
| `/scan` | `sensor_msgs/LaserScan` | Varredura LiDAR             |
| `/odom` | `nav_msgs/Odometry`     | Odometria do Scout Mini     |
| `/tf`   | `tf2_msgs/TFMessage`    | Transformações entre frames |

### Parâmetros Dinâmicos (ROS Params)

Acessíveis via `/amcl` namespace:

```bash
ros2 param set /amcl min_particles 100
ros2 param set /amcl max_particles 2000
ros2 param set /amcl alpha1 0.2  # coeficiente de ruído odométrico rotacional
ros2 param set /amcl alpha2 0.2  # coeficiente de ruído odométrico linear
```

---

## 8. Estrutura do Projeto

```
scout_ws/
├── src/scout_mini_ros2/
│   ├── scout_sim/
│   │   ├── config/nav2_params.yaml     ← Parâmetros AMCL
│   │   ├── launch/scout_sim_maze_nav2.launch.py
│   │   └── worlds/maze_world.sdf
│   ├── scout_description/
│   ├── scout_bringup/
│   └── scout_control/
├── maze_class7_map.pgm              ← Mapa do labirinto
├── maze_class7_map.yaml
├── Docs/
│   ├── Tarefa 1 - Observar a convergencia correta/
│   ├── Tarefa 2 - Testar Pose Inicial errada/
│   ├── Tarefa 3 - Modificar Parametros/
│   └── teste_min_particles_amcl.md
└── README.md (este arquivo)
```

---

## 9. Dicas de Troubleshooting

### AMCL não converge

- Verificar se `/tf` está sendo publicado corretamente
- Aumentar `alpha1`, `alpha2` (ruído odométrico)
- Reduzir `min_particles` para testes rápidos
- Verificar se mapa está carregado: `ros2 topic echo /map --once`

### Gazebo/RViz crash durante execução

- Usar `use_sim_time=true` em todos os nós
- Se problema persista, reduzir `max_particles` ou `map_update_interval`

### TF tree incompleto

- Verificar launch file
- Executar: `ros2 run tf2_tools view_frames`

---

## 10. Referências

- [Nav2 Documentation](https://navigation.ros.org/)
- [AMCL ROS 2](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl)
- [ROS 2 Jazzy Release](https://docs.ros.org/en/jazzy/)
