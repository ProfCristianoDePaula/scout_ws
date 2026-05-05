# Teste Comparativo AMCL: min_particles (100 vs 2000)

Data: 2026-05-05
Workspace: `/home/prof-cristiano/scout_ws`
Pacote recompilado em cada cenário: `scout_sim`

## Objetivo

Comparar dois valores do parâmetro `amcl.ros__parameters.min_particles`:

- Cenário A: `100`
- Cenário B: `2000`

## Procedimento executado

1. Alterar `min_particles` em `src/scout_mini_ros2/scout_sim/config/nav2_params.yaml`.
2. Recompilar com:
   - `source /opt/ros/jazzy/setup.bash`
   - `colcon build --packages-select scout_sim --symlink-install`
3. Executar:
   - `ros2 launch scout_sim scout_sim_maze_nav2.launch.py`
4. Coletar durante execução:
   - `ros2 param get /amcl min_particles`
   - `ros2 topic echo /particle_cloud --once` (timeout de 20 s)

## Resultados

| Métrica                                          | Cenário A (`min_particles=100`) | Cenário B (`min_particles=2000`) |
| ------------------------------------------------ | ------------------------------: | -------------------------------: |
| Build `scout_sim` (resumo colcon)                |                        `0.37 s` |                         `0.42 s` |
| Nó `/amcl` ativo                                 |                             Sim |                              Sim |
| `ros2 param get /amcl min_particles`             |         `Integer value is: 100` |         `Integer value is: 2000` |
| `/particle_cloud --once` em 20 s                 |                    Sem mensagem |                     Sem mensagem |
| Linhas capturadas em `/tmp/particle_cloud_*.txt` |                             `0` |                              `0` |

## Observações relevantes de execução

- Em ambos os cenários, o Gazebo encerrou durante a execução (`Escalating to SIGKILL on [Gazebo Sim Server]`).
- Com isso, não foi possível observar publicação de `/particle_cloud` dentro da janela de coleta.
- Apesar disso, os dois testes confirmaram corretamente a aplicação do parâmetro no nó AMCL em runtime.

## Conclusão

- A alteração de `min_particles` entre `100` e `2000` foi aplicada com sucesso e validada no nó `/amcl`.
- Não houve diferença observável em `/particle_cloud` nesta rodada por limitação de execução do Gazebo (encerramento do servidor antes da coleta).

## Estado final do projeto

- O arquivo de parâmetros foi restaurado para o valor original:
  - `min_particles: 500`
- O pacote `scout_sim` foi recompilado após a restauração.
