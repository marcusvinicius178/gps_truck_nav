# Smac/MPPI: parâmetros efetivos e limites da evidência

Esta revisão preserva o YAML canônico, os experimentos históricos e os valores de
planejamento/controle. O probe lê parâmetros e descritores; não publica comandos,
não envia goals e não modifica parâmetros. Gate 0 e Gate 1 continuam
`NOT_YET_VERIFIED`, com `mission_ready=false`.

## Causa observada e correção da coleta

Na evidência do commit `978ab2328c6cb378d6eea016027192a6cf1dbadf`, o dump CLI do
`/planner_server` terminou com código 124; o dump do `/controller_server` retornou
`{}` com código 0. Os checks estruturados desses nós verificavam apenas
`use_sim_time`. Nenhum dos dois resultados comprova os parâmetros Smac/MPPI.
Também não comprova, isoladamente, um defeito permanente de DDS: os serviços e
os lifecycle nodes aparecem em observadores posteriores da mesma coleta.

`parameter_probe.py` solicita nomes completos diretamente aos serviços
`GetParameters` e `DescribeParameters`, sem depender do resultado de
`ListParameters` nem do parsing de `ros2 param dump`. A listagem é evidência
diagnóstica adicional. O contrato atual contém 40 nomes no planner e 88 no
controller, incluindo todos os campos configurados e defaults relevantes da
versão auditada.

Cada lote tem no máximo 16 nomes, no máximo três tentativas, timeout de 2 segundos
de relógio monotônico e registro de todas as respostas, erros e timeouts. Há no
máximo quatro RPCs em voo e um por combinação nó/método. O executor do probe
principal avança os futures: `poll()` não espera serviço, não faz `spin` e não
usa tempo simulado para limitar a coleta. O fechamento da janela cancela requests
pendentes sem iniciar tentativas novas. Serviço não observado, serviço
indisponível, timeout e resposta malformada permanecem distintos.
O deadline é aplicado também a um future já concluído quando `poll()` o observa:
uma conclusão observada no limite ou depois dele recebe `LATE_RESPONSE`; seu
payload fica em `attempt.response_payload`, sem alimentar valores/descritores
efetivos. A latência registrada é de conclusão observada pelo coletor, não uma
estimativa de latência de rede. Assim, um atraso do executor não produz PASS
retroativo. O probe principal fecha essa coleta antes de aguardar o processo C++.

Há uma diferença relevante do serviço C++ Iron: com
`allow_undeclared_parameters=false`, um único nome não declarado pode causar
`ParameterNotDeclaredException` e uma resposta vazia para o lote inteiro, tanto
em Get quanto em Describe. Por isso os parâmetros obrigatórios têm lotes próprios;
os nomes opcionais, não consumidos ou condicionais são consultados individualmente
depois dos obrigatórios. Resposta vazia permanece `ERROR`, sem fabricar um
`PARAMETER_NOT_SET` ou afirmar qual exceção ocorreu. Não se usa List como pré-requisito.
Essa semântica foi conferida em
[ParameterService rclcpp 21.0.8](https://github.com/ros2/rclcpp/blob/21.0.8/rclcpp/src/rclcpp/parameter_service.cpp)
(blob `09237983396a7e8061a83d4c2c0a4381beb979f3`) e
[NodeParameters](https://github.com/ros2/rclcpp/blob/21.0.8/rclcpp/src/rclcpp/node_interfaces/node_parameters.cpp)
(blob `c8bb62e238e91238d398a787440fa8f6fe3ee51b`).

`collection_status=COMPLETE` significa que todos os lotes explícitos Get/Describe
receberam respostas completas, inclusive os opcionais. O campo separado
`required_collection_status` resume apenas os obrigatórios. Uma resposta
`PARAMETER_NOT_SET`, quando o servidor a devolve por item, é uma resposta
completa; um parâmetro obrigatório ausente ou diferente gera `FAIL` na avaliação.
Resposta parcial, erro ou timeout produz coleta `PARTIAL`. Falha da listagem não
invalida respostas explícitas completas. Nenhum desses status equivale a Nav2
saudável ou autorização para movimento.

## Classes de evidência

| Campo | O que demonstra | O que permanece sem demonstração |
|---|---|---|
| `DECLARED` | `DescribeParameters` devolveu um descritor com tipo declarado | Que um plugin lê o parâmetro |
| `EFFECTIVE` | `GetParameters` devolveu o tipo ROS e o valor esperados, no nó correto | Que o membro interno do plugin recebeu ou usou o valor |
| `SOURCE_CONSUMER_IDENTIFIED` | Há uma leitura no código Nav2 1.2.10 auditado, com namespace e condição compatíveis | Identidade bit a bit entre a biblioteca instalada e a fonte upstream; execução do algoritmo |
| `SOURCE_AUDIT_VERSION_MATCHED` | `package.xml` instalado informa 1.2.10 para o pacote correspondente | Ausência de patches do fornecedor nessa versão |
| `CONFIGURE_LOG_OBSERVED` | O código de configuração imprimiu aquele subconjunto de valores internos | Execução de planejamento, críticos ou controle; valores que não foram impressos |
| `ACTUALLY_USED` | Permanece `NOT_VERIFIED` nesta etapa | Não pode ser promovido a PASS por YAML, descritor, readback ou lifecycle ACTIVE |

São preservados tipos ROS, descritores, valores, nomes exatos, número de tentativa,
latência de resposta e tempos de descoberta. Comparação numérica não confunde
booleanos com inteiros nem inteiros com doubles. `GridBased.*` lido no controller
não substitui evidência do planner. `GridBased.non_straight_penalty=2.6` no fixture
de regressão falha contra o valor canônico 2.5; nenhum desses testes altera um nó
em execução.

O arquivo `plugin_parameter_contract.py` registra URLs e SHAs dos blobs upstream
auditados. A proveniência instalada registra `package.xml`, XMLs pluginlib e hashes
SHA-256 das bibliotecas Smac/MPPI; também registra as versões dos servidores
planner/controller. Ela declara explicitamente `INSTALLED_FILES_ONLY_NOT_PROCESS_MAPPINGS`:
encontrar um arquivo não prova que esse arquivo está carregado no processo.
Versão ausente/divergente suspende a atribuição de consumer como
`UNKNOWN_INSTALLED_VERSION`; ID de plugin divergente produz
`UNKNOWN_PLUGIN_IDENTITY`. O tipo e o valor efetivos continuam observáveis.
O assessor cruza ainda esses hashes com os arquivos mapeados registrados em
`provenance.runtime_binary_identity`: `smac_hybrid` e `mppi_controller` precisam
corresponder às bibliotecas dos respectivos prefixes instalados. Ausência de
associação suspende o consumer como `UNKNOWN_BINARY_ASSOCIATION`; hashes
divergentes produzem `BINARY_IDENTITY_MISMATCH` e check `FAIL`. Identificar um
arquivo em cada lugar separadamente não satisfaz essa correlação. Os resultados
ficam em `binary_associations` e nos checks `plugin_binary_association:<nó>`.

## Contrato auditado em Nav2 1.2.10

| Grupo | Namespace e campos cobertos |
|---|---|
| Seleção e servidores | `planner_plugins`, `controller_plugins`, plugins selecionados, frequência esperada do planner, frequência do controller, limiares de velocidade, failure tolerance, relógio |
| Smac Hybrid | `GridBased`: downsampling, tolerância, unknown, limites de iterações/tempo, modelo cinemático, bins, raio mínimo, lookup/cache, penalidades, expansão analítica suportada, debug e smoother |
| MPPI | `FollowPath`: horizonte, dt, batch, iterações, temperatura/gamma, limites e desvios de velocidades, retry, modelo, regeneração de ruído, path handler e AckermannConstraints |
| Críticos ativos | Seleção, enabled, pesos, potências, offsets, distâncias, ocupação, orientação, modo, geometria/colisão e inflation layer dos seis críticos selecionados |
| Defaults explícitos na leitura | `GridBased.smoother.*`, `FollowPath.vy_std`, `FollowPath.TrajectoryVisualizer.{trajectory_step,time_step}`, `FollowPath.PathAlignCritic.trajectory_point_step` |

Os defaults são expectativas de readback com origem
`UPSTREAM_1_2_10_DEFAULT`. O probe não os escreve. Todos os campos do YAML são
capturados, inclusive configurações inativas e nomes sem consumer identificado.

| Entrada canônica | Resultado da auditoria 1.2.10; ação nesta revisão |
|---|---|
| `planner_patience` | Sem leitura no planner server/Smac auditado; preservar e bloquear seleção como knob útil |
| `GridBased.transform_tolerance`, `GridBased.costmap.*` | Não configuram o objeto costmap separado nem são lidos nesse Smac; os dumps efetivos das costmaps já corrigidas continuam sendo a evidência apropriada |
| `GridBased.analytic_expansion_max_cost`, `analytic_expansion_max_cost_override`, `allow_primitive_interpolation` | Sem read site nessa versão; não importar pressupostos de releases posteriores |
| `FollowPath.wz_min` | O optimizer usa limite simétrico derivado de `wz_max`; esta entrada não fornece um segundo limite |
| `FollowPath.reset_period` | Sem read site no MPPI 1.2.10 inspecionado |
| `FollowPath.trajectory_step`, `FollowPath.time_step` | Read sites reais usam `FollowPath.TrajectoryVisualizer.*`; consultar ambos os nomes, preservar o YAML e não alegar consumo dos nomes planos |
| `FollowPath.ObstaclesCritic.cost_scaling_factor`, `inflation_radius` | O crítico consulta os accessors da InflationLayer selecionada da costmap local; os valores do crítico no YAML não são a fonte |
| `FollowPath.inversion_xy_tolerance`, `inversion_yaw_tolerance` | Leitura condicionada a `enforce_path_inversion=true`; atualmente false: `CONDITION_DISABLED` |
| `general_goal_checker.*` | Configuração inativa: o goal checker selecionado é `front_cabin_goal_checker` |

O `ObstaclesCritic` consulta, portanto, a inflação local efetiva (no YAML atual,
`cost_scaling_factor=0.5`, `inflation_radius=3.0`), não os valores 5.0/8.0 da sua
subárvore. A evidência de costmap deve confirmar esses accessors; não se altera
inflação para fabricar concordância. `vy_max` e `vy_std` têm read sites, mas isso
não implica movimento lateral no modelo Ackermann. Os parâmetros do visualizador
são lidos no configure mesmo com `visualize=false`; publicação de trajetórias
continua desativada.

O `bringup.log` de 978ab23 contém sete observações reconhecidas pelo parser:
frequência 20 Hz, pesos/potências PathAlign/PathAngle/PreferForward/Goal,
pesos/potência de Obstacles e limites/tolerância/modelo Smac. Na fonte Iron,
PathAlign imprime o nome `ReferenceTrajectoryCritic`; a normalização registra o
nome original e a versão auditada. Os logs não imprimem batch, horizonte ou
`non_straight_penalty`; o parser não inventa esses valores. Ele não promove nem
altera o diagnóstico principal.

Fontes primárias usadas: [Smac Hybrid](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_smac_planner/src/smac_planner_hybrid.cpp),
[parâmetros do smoother](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_smac_planner/include/nav2_smac_planner/types.hpp),
[MPPI optimizer](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/optimizer.cpp),
[ParametersHandler](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/include/nav2_mppi_controller/tools/parameters_handler.hpp),
[PathHandler](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/path_handler.cpp),
[TrajectoryVisualizer](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/trajectory_visualizer.cpp),
[PathAlignCritic](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/critics/path_align_critic.cpp)
e [ObstaclesCritic](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/critics/obstacles_critic.cpp).
O mapa completo dos arquivos e SHAs está no contrato emitido.

## Percepção: fonte externa ausente

`/external_area_pointcloud` tem assinantes nas camadas de obstáculos global/local
e no RViz, mas a evidência 978ab23 não contém publisher. A presença na lista de
tópicos não prova um produtor. A busca no código e no snapshot histórico encontra
consumidores/configuração, sem implementação identificada de um produtor para esse
nome. Isso deixa a procedência da fonte externa sem resolução: não permite concluir
que ela seja Virtual Lanes ou que seja uma fonte física conhecida.

`nav_virtual_lanes/wall_costmap_generator.py` publica `virtual_wall_points`, outro
tópico. Esta revisão não habilita Virtual Lanes, não remapeia pontos virtuais para
uma entrada de obstáculos e não remove a fonte ausente das camadas para obter PASS.

O `/sick_scan_2D` teve 296 mensagens; a última amostra tinha 360 ranges e zero
retornos finitos. Todos `+inf` podem representar uma cena válida sem alvo no alcance.
Isso não demonstra detecção de obstáculos, tampouco prova sozinho defeito no sensor.
A configuração preservada usa `inf_is_valid=false`; não reinterpretar infinito como
obstáculo confirmado nem alterar a política de clearing nesta revisão.

## Critérios de revisão e próxima evidência

1. Get/Describe completos para todos os lotes obrigatórios, com valores, tipos,
   plugin IDs e namespaces correspondentes; timeout e `{}` nunca satisfazem isso.
2. Proveniência de versões/bibliotecas identificada; divergência de versão exige
   nova auditoria da fonte correspondente. Nomes não consumidos, inativos ou
   condicionais devem ser explicitamente excluídos do futuro espaço de tuning ou
   reconciliados numa variante revisada, preservando o histórico.
3. Logs de configure associados à mesma execução servem apenas para o subconjunto
   impresso. Prova adicional de consumo interno requer instrumentação revisada;
   prova de efeito no algoritmo exige ensaio futuramente autorizado. Nenhuma
   intervenção de parâmetro ou computação de trajetória é executada agora.
4. Para percepção, identificar implementação e configuração do produtor esperado,
   confirmar publisher/QoS, frame, transformação e frescor. Antes da aceitação,
   um ensaio estático com alvo conhecido deverá demonstrar retornos e marcação/
   clearing coerentes nas costmaps. Isso exige escopo aprovado separado; a fonte
   ausente e a ausência atual de prova de obstáculo continuam bloqueios.
5. A aprovação do revisor continua necessária antes de qualquer autorização de
   movimento; esta coleta de parâmetros não resolve TF, localização ou atuação.

Integração no probe principal:

```python
collector = PluginParameterProbe(node, canonical)
# Dentro da janela existente; o executor principal continua atendendo futures.
collector.poll()
raw = collector.finish()
provenance = installed_plugin_provenance()
provenance['runtime_binary_identity'] = observed['runtime_binary_identity']
assessment = assess_plugin_parameters(raw, provenance)
```

Após a captura do `bringup.log` e cleanup, extração opcional somente de leitura,
num arquivo novo do diretório exclusivo da execução (o `noclobber` evita sobrescrita):

```bash
set -o noclobber
python3 tools/auto_tuning/parameter_probe.py \
  --configuration-log "$PHASE_A_PROBE_DIR/bringup.log" \
  > "$PHASE_A_PROBE_DIR/plugin_configure_log_observations.json"
```

O runbook principal define o diretório exclusivo e os comandos completos de
build/probe. Regressões locais, sem ROS:

```bash
python3 -m unittest discover -s tests -p 'test_plugin_parameter_contract.py' -v
```
