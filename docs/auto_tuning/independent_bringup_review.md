# Revisão causal do bringup — Fase A

**Gate 0 e Gate 1: NOT YET VERIFIED. Missões e otimização bloqueadas.** As correções de carregamento e BT abaixo estão implementadas para a próxima imagem; sua aprovação depende dos parâmetros e estados efetivos do próximo probe. Não houve execução de ROS/Gazebo neste Work.

## Evidência examinada

`nav2_phase_a_probe_evidence.zip`: 110 arquivos, SHA-256 `6a48d2c9e96d8ca9202763dcb5f16250591afbbc598937c40b0fe936f6972034`. O ZIP original permanece separado do repositório; as referências abaixo são caminhos internos desse arquivo.

| Identidade | Observado |
|---|---|
| Commit | `b8922aee9e792b357dc15263a256e11a08cdbfb1` |
| Image ID | `sha256:8fef65a928d1e8cda20724dfb24404d3396755c3d545170b52b7e3d0acc3ac16` |
| Build | `IMAGE_VALIDATED`, cleanup concluído |
| Probe | `OBSERVATIONS_COLLECTED`, cleanup concluído |
| Janela do probe | 29/10/2026, 18:51:56–18:53:40, São Paulo |
| Pacotes | Nav2 1.2.10; launch_ros 0.24.2; Gazebo Classic 11.10.2; gazebo_plugins 3.7.0; robot_localization 3.7.1 |

`OBSERVATIONS_COLLECTED` informa que os comandos de coleta e o encerramento terminaram. Os próprios logs mostram três lifecycle nodes inativos e erros de configuração. Portanto esse resultado nunca foi aprovação de saúde integral.

## 1. Causa das costmaps padrão

O YAML canônico contém os parâmetros esperados sob `global_costmap/global_costmap/ros__parameters` e `local_costmap/local_costmap/ros__parameters`. Os dumps `probe/runtime_probe/parameters_{global,local}_costmap.log`, porém, contêm 5×5 m, resolution 0.1, rolling false, footprint vazio, base_link e StaticLayer. O log do bringup contém exatamente 1004 avisos de robot out of bounds e 101 de ausência de mapa na StaticLayer local.

O problema está no caminho de composição. `navigation_launch.py` usa `ParameterFile(RewrittenYaml(..., root_key=namespace))` nos ComposableNodes. No `launch_ros` Iron, `LoadComposableNodes` seleciona os parâmetros do nó solicitado e os envia no serviço de carga. O planner e o controller são os nós solicitados; suas costmaps são nós filhos criados em C++, com `NodeOptions` próprios. Os blocos das costmaps não são automaticamente transmitidos por aquele pedido de carga.

No full launch do projeto, `component_container_isolated` recebia somente `use_sim_time`. Faltava o YAML completo nos argumentos globais do processo, de onde os nós filhos obtêm sua configuração. O `bringup_launch.py` oficial do Nav2 1.2.10 passa `configured_params` também ao container.

**Correção:** `gps_waypoint_follower.launch.py` passa ao container um `ParameterFile` com o YAML completo selecionado. O include de navegação continua recebendo a mesma fonte. O YAML canônico e seus valores não mudam. O full simulator usa namespace raiz como antes; não se anuncia suporte a namespacing integral de sensores/TF.

Há três níveis distintos de verificação:

1. Regressões offline verificam o wiring real do launch com construtores inertes, a hierarquia YAML e a preservação do contrato canônico.
2. `verify_parameter_transport.py`, executado pelo smoke dentro da imagem, usa as APIs instaladas de `RewrittenYaml`, `ParameterFile` e expansão do Node para conferir o arquivo completo nos argumentos do container, opt-in BT e hierarquia com namespace. A seleção de parâmetros dos componentes pais não deve ser confundida com o parser dos nós filhos. Esse check não inicia os servidores e não prova parâmetros aplicados em runtime.
3. O próximo probe consulta os parâmetros dos nós vivos e salva valores esperados/observados e divergências em `readiness.json`. Os dumps CLI permanecem disponíveis para inspeção independente.

| Contrato efetivo esperado | Global | Local |
|---|---|---|
| Dimensões em metros | 1000×300 | 200×120 |
| Resolução | 1.0 | 1.0 |
| Rolling window | true | true |
| Frame global / base | map / base_footprint | odom / base_footprint |
| Footprint | Quatro vértices do YAML, ±4.8025 e ±1.739 | Mesmo footprint |
| Plugins | obstacle_layer, inflation_layer | obstacle_layer, voxel_layer, inflation_layer |

As observações e parâmetros de cada camada devem corresponder ao YAML, sem StaticLayer. A fonte `/external_area_pointcloud` continua configurada no YAML e sem publisher no bringup examinado. Isso deve aparecer como dependência ausente no diagnóstico; não habilitar Virtual Lanes nem apagar essa dependência para obter um PASS aparente.

## 2. Causa do BT inativo e correção prospectiva

O dump de `bt_navigator` seleciona os XMLs padrão de `/opt/ros/iron`, que contêm `Spin`. O YAML canônico omite a biblioteca BT de Spin e o behavior de giro está desabilitado. `probe/bringup.log` registra `Node not recognized: Spin` ao carregar o XML padrão de NavigateToPose. A ativação do navigator falha e o lifecycle manager não chega a ativar waypoint_follower e velocity_smoother.

Foram adicionados dois XMLs em `config/auto_tuning/behavior_trees/`, derivados dos defaults da versão 1.2.10, para NavigateToPose e NavigateThroughPoses. Removem Spin e BackUp; preservam os IDs GridBased/FollowPath, blackboards, replanning e recuperações de limpeza/Wait limitadas. Não se habilita rotação no lugar, movimento de recuperação ou edição em `/opt/ros/iron`.

`use_ackermann_bt:=true` é um opt-in do launch, selecionado pelo helper do probe. Apenas os dois paths `default_nav_to_pose_bt_xml` e `default_nav_through_poses_bt_xml` são reescritos em uma fonte temporária compartilhada por container/componentes. Chamadas antigas mantêm o opt-in desativado.

O próximo probe exige os sete lifecycle nodes necessários ACTIVE e os dois navigators/path XML efetivos corretos. Em Nav2 1.2.10, a ativação de cada `BtActionServer` carrega seu XML antes de ativar o action server; o `bt_navigator` só conclui a ativação após ativar todos os navigators configurados. Essa evidência valida a carga dos XMLs sem enviar goals. Não prova execução de uma missão.

## 3. Localização e atuação: diagnóstico ampliado, liberação pendente

Ver `localization_actuation_bringup_review.md` para a análise completa, fontes e limites. Os metadados GNSS/IMU mascaram lever arms existentes, covariâncias GPS/orientação IMU exigem uma política explícita, e o probe anterior não chamou `/fromLL` nem mediu `/clock` em uma janela. Não foram alterados datum, SDF/URDF, ruído, geometria ou YAML histórico para aproximar resultados.

O volante visual ausente é opcional no plugin 3.7.0 e não é a causa do BT inativo. A diferença MPPI yaw rate versus esterçamento do plugin permanece um bloqueador real de atuação. Nenhum conversor baseado em wheelbase presumida foi introduzido. Não se enviam Twist, goals ou chamadas de movimento para investigar essas condições.

## 4. Prontidão passa a ser uma evidência separada

O coletor mantém os logs CLI e adiciona uma janela limitada com serviços de parâmetros/lifecycle e assinaturas de dados. Salva `readiness_observations.json` e `readiness.json`, com resultados PASS/FAIL/UNKNOWN/BLOCKED_REVIEW por critério, parâmetros esperados/observados, relógio, frescor, covariâncias, frames, conversões `/fromLL` e metadados das costmaps. Além da idade em tempo simulado, exige atualização/avanço recente de `/clock` medido pelo relógio monotônico de parede; clock e sensores parados juntos não podem aparentar frescor. TF dinâmico exige mais de uma amostra com stamps avançando; TF estático aceita stamp zero. Os limiares explícitos de diagnóstico não são critérios de aceitação experimental.

O observador TF em C++ usa o GID de mensagens realmente recebidas por aresta e registra endpoints. Isso evita confundir quatro publishers de `/tf` com quatro autoridades do mesmo transform. `/tf_static` usa QoS reliable/transient local. A unicidade observada vale somente para a janela amostrada; ausência de evidência ou emissor desconhecido não aprova o grafo. O caminho Python de `MessageInfo` no Iron não expõe esse GID, motivando o observador C++ somente de leitura.

O helper mantém `status=OBSERVATIONS_COLLECTED` exclusivamente para coleta/cleanup, acrescenta `diagnostic_status` e `readiness_collection_status` no status e no resumo de terminal, e mantém `mission_ready=false`. Relatório ausente/inválido ou captura FAILED fazem a tentativa falhar sem pular cleanup. `BLOCKED`, `INCOMPLETE` ou `REVIEW_REQUIRED` são conclusões diagnósticas; nenhuma significa prontidão para missão. Os Gates permanecem não verificados em todos os caminhos.

A correção precisa de novo build e probe no ipt-sim-01. APT pode reutilizar cache se a base continuar igual; alterações de launch recompilam/reinstalam o pacote, e o observador TF é compilado dentro da imagem. O runtime continua sem rede do host, sem GPU/dispositivos/volumes compartilhados e com cleanup somente dos containers próprios. O runbook contém os comandos completos e um novo diretório exclusivo de evidências.

## Verificação executada neste Work

**142 testes offline PASS**, incluindo fixture dos dumps reais reprovados, contratos de transporte de parâmetros, ambas as BTs, falhas de coleta, preservação de evidências e compilação/testes do núcleo C++ de observações TF. Auditoria estrutural PASS: 59 arquivos Python/launch, 15 YAMLs e 37 XMLs. Nove blocos Bash do runbook, dois blocos Python embutidos e scripts de imagem/coleta tiveram sintaxe verificada. Os quatro hashes canônicos protegidos permanecem iguais, assim como o checkout de análise.

Logs e proveniência: `evidence/bringup_review_tests.log`, `evidence/bringup_review_checks.log` e `evidence/bringup_review_summary.json`. Os logs anteriores foram mantidos. Não foi executado Docker/ROS neste Work: o adaptador ROS do observador, o smoke usando Iron instalado e a aplicação efetiva dos parâmetros continuam pendentes da nova imagem/probe no host.

## Fontes do carregamento e dos BTs

- [Nav2 1.2.10 bringup_launch](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_bringup/launch/bringup_launch.py): parâmetros completos no container.
- [launch_ros 0.24.2 LoadComposableNodes](https://github.com/ros2/launch_ros/blob/0.24.2/launch_ros/launch_ros/actions/load_composable_nodes.py) e [to_parameters_list](https://github.com/ros2/launch_ros/blob/0.24.2/launch_ros/launch_ros/utilities/to_parameters_list.py): seleção do pedido de carga por nó/namespace.
- [Costmap2DROS 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_costmap_2d/src/costmap_2d_ros.cpp): opções do nó filho e defaults.
- [RewrittenYaml 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_common/nav2_common/launch/rewritten_yaml.py) e [ParameterFile Iron](https://github.com/ros2/launch_ros/blob/0.24.2/launch_ros/launch_ros/parameter_descriptions.py).
- [NavigateToPose default](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml) e [NavigateThroughPoses default](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml).
- [BtActionServer](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_server_impl.hpp) e [BtNavigator](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_bt_navigator/src/bt_navigator.cpp): carga e ativação das árvores.
