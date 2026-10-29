# Environment preflight — 29/10/2026

## Decisão

**HOST PREFLIGHT APROVADO para ipt-sim-01; reprodutibilidade do runtime ainda não demonstrada.** Gate 0 e Gate 1: **NOT YET VERIFIED / REQUIRES LOCAL EXECUTION**. Existe caminho plausível com Docker/Iron/Classic, mas não há base para anunciar build, navegação ou otimização bem-sucedidos.

O relatório `auto_tuning_feasibility_report.md`, de 14/10/2026, foi localizado e lido integralmente. Sua decisão CONDITIONAL GO permanece: os requisitos de execução e reset ainda estão abertos. Nenhum bag histórico, tese ou Overleaf foi modificado.

## Baselines e mudança desde 14/10

| Componente | Revisão auditada | Resultado |
|---|---|---|
| Integração pública | `19f38e535bebc8a03a6aeef75c29160e0524668b` | Mesmo commit do relatório anterior; último commit em 24/09/2026. |
| Métricas | `36ba76823cc7a124540412cf3ce09c73e6cd1d05` | Mesmo commit; checkout permanece intacto. |
| Nav2 opcional source | `022e7e18570d75a969b96ccd1c3d5c5c443b3f12` | Declarado em `nav2_gps.repos`; não é usado pela receita binária e não representa o fork histórico privado. |
| Branch de preparação | `engineering/2026-10-preflight-gates` | Alterações prospectivas isoladas; nenhuma modificação direta em master. |

O avanço desta iteração é código de preparação, auditoria específica de incompatibilidades e recuperação verificada dos assets. Não são novos resultados experimentais. As referências a linhas canônicas abaixo correspondem à base acima.

## Estado das evidências

| Classificação | Evidência |
|---|---|
| VERIFIED BY EXECUTION | Clone dos dois repositórios, `git lfs pull`, `git lfs fsck`, audit estrutural, testes Python e diagnóstico real deste Work. |
| PROVIDED HOST EVIDENCE / REVIEWER APPROVED | JSON de ipt-sim-01 executado pelo proprietário, lido integralmente; resumo em `host_preflight_ipt_sim_01.md`. Não é execução remota pelo agente. |
| VERIFIED BY STATIC INSPECTION | Launch headless, dependências, configuração EKF/TF, plugin custom, schema do avaliador, source Iron dos knobs e filtros. |
| PROPOSED BUT NOT TESTED | Imagem Docker nova, overlay de localização, opt-out do TF legacy em ROS, probes no container, candidatos A/B e extração de MCAP real em Iron. |
| BLOCKED / REQUIRES LOCAL EXECUTION | Pull da imagem-base, resolução APT, build C++, plugin loading, sensores/TF/lifecycle, missão, gravação, métricas reais e reset automático. |

## Docker e distribuição

O Dockerfile canônico parte de `osrf/ros:iron-desktop-full` sem digest e instala pacotes sem versões. Compose mantém `sleep infinity`, monta fonte em `/repo:ro` e persiste `/ws/build`, `/ws/install` e `/ws/log`. Portanto `docker compose up` **não** compila nem inicia a simulação; volumes antigos podem reintroduzir um overlay anterior pelo `entrypoint.sh`.

O `.dockerignore` canônico inclui somente Dockerfile/entrypoint. A nova receita `docker/auto_tuning/Dockerfile` possui ignore próprio, recebe `ROS_BASE` por digest, copia as fontes e compila somente `truck_bringup` dentro da imagem. Inclui `ros-iron-rosbag2-storage-mcap`, explicitamente ausente da receita original. A execução proposta usa container novo, loopback próprio, sem rede externa e sem volumes de build compartilhados. Não precisa instalar ROS no host.

**Limite de reprodutibilidade:** um digest de base fixa apenas a base. APT ainda será resolvido no build de descoberta; registrar versões e salvar a imagem final por image ID é obrigatório antes de comparar candidatos. Ainda falta lock/preservação dos pacotes para garantir rebuild independente. Não foi possível verificar o pull ou APT aqui; existência de Dockerfile ou metadado de release não prova disponibilidade dos binários.

Iron e Gazebo Classic 11 estão EOL. O alvo userspace continua Ubuntu 22.04/Iron/Classic; não ocorreu migração. O índice rosdistro registra Nav2 1.2.10-1 e gazebo_ros_pkgs 3.7.0-3, sem comprovar a versão que um APT real instalará. Comparar `dpkg-query`, prefixos ROS e bibliotecas realmente carregadas; não importar Nav2 source sobre pacotes binários sem uma decisão explícita de baseline.

## Dependências efetivas

| Camada | Componentes necessários |
|---|---|
| Build | C++ toolchain, CMake, colcon, ament_cmake/python, nav2_common/core/costmap_2d, rclcpp/lifecycle, geometry_msgs, pluginlib, angles, tf2/tf2_geometry_msgs. |
| Simulação | Gazebo Classic 11, gazebo_ros/gazebo_plugins; Ackermann, GPS, IMU, ray scan e câmera; assets `sun`/`ground_plane` instalados. |
| Navegação/localização | Nav2 Smac/MPPI/BT/behaviors/smoother/waypoint/lifecycle/velocity smoother, nav2_simple_commander, robot_localization, robot_state_publisher, tipos ROS e TF. |
| Plugin local | `truck_bringup/FrontCabinGoalChecker`, biblioteca `front_cabin_goal_checker`; compilar contra a API/ABI binária realmente instalada. |
| Registro/avaliação | rosbag2, rosbag2_py, storage MCAP, rclpy, rosidl_runtime_py, NumPy e tipos das mensagens. PyYAML para preparação e metadados. |
| GUI opcional | RViz2, Mapviz/plugins/tile_map/swri_transform_util, Tk. Sem GUI não significa necessariamente sem renderização dos sensores. |

O plugin custom de goal usa defaults no C++ (xy 1,5 m, yaw 0,25 rad, offset 5 m); não assumir que o bloco YAML inativo de outro checker o configura. Virtual Lanes não participa do build inicial.

## Bloqueios concretos e preparação

| Achado estático | Consequência e tratamento |
|---|---|
| Plugin Gazebo Ackermann 3.7.0 atribui angular.z ao esterçamento, aplica clamp e tan/atan; MPPI publica yaw rate. O caminho público liga cmd_vel_nav→smoother→cmd_vel sem conversor identificado. | **Bloqueador de missão.** Conferir pacote/biblioteca realmente instalada e implementar conversão com geometria validada, sinais de ré, saturação, zero speed e timeout antes de dirigir. Não aplicar fórmula com wheelbase presumida. Não prova erro no fork privado histórico. |
| Ambos os EKFs usam `world_frame: map`, `publish_tf: true` (`dual_ekf_navsat_params.yaml:10,15,69,75`). | Duas autoridades possíveis para map→odom. Overlay prospectivo faz EKF local world=odom, publish_tf=false; Gazebo já publica odom→base_footprint. Global continua map→odom. |
| EKF global usa `odom1: gps` e habilita velocidades (`:89–94`), enquanto navsat produz `odometry/gps` com posição. | Overlay liga `/odometry/gps` e apenas x/y. Preserva datum, covariâncias, two_d_mode e parâmetros físicos; verificar convergência e frames. |
| Static cloud TF do full launch tem origem zero/yaw 90°, distinto do joint fixo URDF. | Argumento novo `publish_legacy_cloud_tf:=false` permite usar apenas URDF. Default antigo permanece true. Geometria do URDF é compatível estaticamente com posição/orientação do scan SDF. |
| GPS SDF possui duas remaps `~/out`, sem `frame_name` explícito; antena deslocada (3.8205,0,3.062). Upstream usa parent link como default de frame. | Confirmar tópico/header e TF da antena: declarar base_footprint para sensor deslocado mascara o lever arm no navsat. IMU também requer conferir frame. Não retunado automaticamente. |
| Spin desabilitado no YAML, BT padrão da versão instalada não congelado. | Verificar XML carregado e actions requeridas. Se contiver Spin, criar BT prospectivo adequado ao Ackermann antes da missão; não habilitar giro no lugar. |
| MPPI `vx_max=25.0`, smoother `max_velocity[0]=8.33`, SDF `max_speed=20`. | Não equivalem automaticamente à condição nominal 20 km/h. Verificar unidades, limites e velocidade realizada; não alterados nesta fase. |
| `/external_area_pointcloud` configurado em costmaps sem publisher no full launch. | Verificar se a camada fica current e se o missing topic bloqueia readiness. Não tratar ausência de dados como ausência de obstáculos. |
| Sender espera serviço/ativação/resultado sem teto global; aplica map-x +4,8025 m. | Supervisor deverá limitar tempos e separar action status, progresso e métricas; congelar alinhamento da rota. |
| Missão ativa pública diz `SCENARIO 5_6 / Straight line + ARC 40`. | Não associar automaticamente aos CSVs S1–S3. Gate de avaliação exige associação rota/referência documentada. |
| Avaliador escolhe somente primeiro MCAP e pode terminar 0 após bag_read_failed. | Novo adapter rejeita split bag e trata a linha de erro como measurement failure. |

O overlay completo em `config/auto_tuning/dual_ekf_navsat_gate0.yaml` modifica exatamente quatro campos do YAML de localização. É um candidato de reparo, não um baseline já validado. Reavaliar consistência odometria/TF, pose inicial, drift, frames, covariância e tempo antes de enviar goals.

## Assets e headless

Sete meshes Git LFS, payload total 277.750.112 bytes (~264,9 MiB). O checkout inicial estava com ponteiros; nesta iteração `git lfs pull` **concluiu**, `git lfs fsck` **passou** e o audit estrutural passou após hidratação. Isso comprova acesso a esses objetos neste Work nesta data; não testa carregamento Gazebo nem garante a rede do host. Nove referências do modelo ativo usam cinco dessas meshes únicas. Texturas PNG são blobs Git normais.

Headless já existe: `use_gzclient:=false use_rviz:=false use_mapviz:=false use_xcb:=false`. O launch usa diretamente `gzserver ... -s libgazebo_ros_init.so`; não propaga automaticamente os argumentos do launch oficial `gazebo_ros`. Assim, suporte upstream a seed não significa que nosso entrypoint já o exponha. Câmera pode exigir backend OpenGL; confirmar seus erros e tópicos. Não há seed MPPI comprovadamente configurável no código examinado; registrar seed como não controlada e medir variabilidade depois do Gate 1.

## Parâmetros e métricas

Os oito nomes propostos existem no SHA Nav2 opcional inspecionado, mas os limites seguros e os binários efetivos ainda exigem validação. Para o micro-PoC foi preparado somente `GridBased.non_straight_penalty`, **2,5 versus 2,6**, mantendo os raios Smac/MPPI em 11,6 m. O gerador salva YAML completo A/B e checksums; não executa testes de navegação nem certifica segurança.

No MPPI 1.2.10 inspecionado, ObstaclesCritic obtém raio/escala da InflationLayer; campos `ObstaclesCritic.inflation_radius` e `cost_scaling_factor` do YAML não controlam esses valores nesse código. Dump ROS mostra declaração, mas não basta para afirmar influência no algoritmo.

A avaliação core preserva PR≥0,90 para geometric success, PR≥0,70 para mission eligibility e cobertura≥0,80 para tracking, sem substituir falhas de infraestrutura por scores ruins. O adapter fixa commit/hash do avaliador, exige referência/rota/associação explícitas e deixa action_status separado. Seis CSVs XY S1/S2/S3 × vel15/20 estão disponíveis. Raw-GPS humanos e waypoints auxiliares da camada humana não estão completos; a camada fica desativada. O equivalente geométrico default do avaliador é 6,804 m e precisa ser conciliado com o modelo; não substituir silenciosamente por 5,950 m. O README das ferramentas de bag indica desenvolvimento em Jazzy; leitura real no Iron ainda precisa de teste.

## Host e capacidade deste Work

**Host alvo aprovado:** ipt-sim-01 tem Ubuntu 22.04.5 x86_64, 32 CPUs lógicas, 60,3 GiB RAM (40,8 disponíveis na coleta), 366,6 GiB livres no filesystem de Downloads, Docker 29.8.2/Compose 5.5.1 e Git/LFS existentes. Daemon local respondeu. Humble permanece intacto. Ausência de Gazebo no PATH do host não exige instalá-lo fora da imagem. NVIDIA/DRM presentes não provam renderer operacional. O próximo helper mede separadamente o disco do Docker; os 366,6 GiB não se aplicam automaticamente a ele.

Diagnóstico observado: Ubuntu 24.04.3 x86_64, quota cgroup 8 cores/8 GiB; sem Docker CLI/daemon, ROS, Gazebo, CMake ou `/opt/ros`. Não existe acesso à máquina Linux de Marcus. Disponibilidade de CPU e disco por si só não fornece o runtime requerido.

Requisitos funcionais do host: Docker Linux local, amd64 inicialmente, permissão de execução de containers, internet para build/LFS, disco para imagem/modelos/bags e backend de renderização se os sensores precisarem. Não é necessário servidor X para clientes desativados, mas servidor de simulação ainda precisa ser testado.

**Mínimo exato de RAM/CPU/disco não foi medido.** Referência de planejamento: 16 GiB/4+ cores/50 GiB livres; preferir 32 GiB/8 cores/100 GiB para folga e MCAP. Esses valores não garantem MPPI a 20 Hz nem desempenho em tempo real. O piloto deve registrar pico RSS, disco/minuto, frequência e jitter de controle, real-time factor, duração startup/mission/teardown e throttling.

## Caminho mais curto e fontes

Host/Docker/LFS já diagnosticados e aprovados → `local_phase_a.py build` em checkout novo → smoke checks da imagem → `local_phase_a.py probe` com Xvfb/Mesa e rede isolada → revisão TF/sensores/BT e resolução da interface yaw-rate/esterçamento → congelar missão/referência/velocidade → Gate 0 com bag/métricas → implementar e executar o supervisor de duas configurações → parar para revisão independente. A preparação não elimina essas dependências. O runbook não instala nada no host nem reutiliza CARLA/Autoware.

Fontes primárias:

- [Integração na base auditada](https://github.com/marcusvinicius178/gps_truck_nav/tree/19f38e535bebc8a03a6aeef75c29160e0524668b).
- [Pipeline de análise congelado](https://github.com/marcusvinicius178/nav2_paper_scripts/tree/36ba76823cc7a124540412cf3ce09c73e6cd1d05).
- [Nav2 source opcional](https://github.com/ros-navigation/navigation2/tree/022e7e18570d75a969b96ccd1c3d5c5c443b3f12), especialmente inflation_layer, smac_planner_hybrid, obstacles_critic e noise_generator.
- [robot_localization Iron](https://github.com/cra-ros-pkg/robot_localization/tree/78d3c9d2522f645497fb5e8bc0ef613216c0fa82), dual_ekf_navsat_example e navsat_transform.
- [Ackermann Gazebo 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp), linhas 513–521 e 551–555; [frame default](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_ros/src/utils.cpp).
- [Iron EOL](https://discourse.ros.org/t/iron-irwini-is-end-of-life/41031), [Gazebo Classic EOL](https://classic.gazebosim.org/), [índice Iron](https://github.com/ros/rosdistro/blob/master/iron/distribution.yaml).
- [Docker Ubuntu](https://docs.docker.com/engine/install/ubuntu/), [receita OSRF Iron](https://github.com/osrf/docker_images/blob/master/ros/iron/ubuntu/jammy/desktop-full/Dockerfile).

Os documentos e testes são avaliados em conjunto com `engineering_handoff.md`; nenhum PASS de ferramenta offline é apresentado como PASS do simulador.
