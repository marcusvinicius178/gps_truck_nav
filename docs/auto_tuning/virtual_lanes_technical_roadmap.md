# Virtual Lanes — roteiro técnico independente

Data: 29/10/2026. Base auditada: `gps_truck_nav@19f38e535bebc8a03a6aeef75c29160e0524668b`.

**Decisão:** preservar Virtual Lanes como frente independente do auto-tuning. O código contém protótipos úteis, mas não demonstra restrição espacial, escolha de três faixas ou mudança de faixa executável pelo caminhão. A próxima implementação deve priorizar geometria única de corredores e filtros nativos de costmap, mantendo Smac Hybrid-A* e MPPI. Este documento não implementa essa frente e não habilita seus nós no baseline.

O relatório `auto_tuning_feasibility_report.md`, de 14/10/2026, foi usado como ponto de partida. O commit público permanece o mesmo. A presente inspeção detalha riscos já apontados; não constitui evidência de novo experimento nem altera as 212 execuções históricas.

## 1. Estado da evidência

| Classificação | Resultado |
|---|---|
| VERIFIED BY STATIC INSPECTION | Geradores, detector, visualizadores, mission sender, plugin atual, variante histórica e YAML foram lidos. Fontes Nav2 do SHA opcional foram consultadas. |
| VERIFIED BY EXECUTION | Nenhum resultado ROS/Gazebo de Virtual Lanes. A inspeção de arquivos não satisfaz essa categoria. |
| PROPOSED BUT NOT TESTED | Arquitetura, contratos, sequência de implementação e critérios abaixo. |
| BLOCKED / REQUIRES LOCAL EXECUTION | Build de plugins, parâmetros efetivamente carregados, integração dos filtros, footprint completo, transições, obstáculos e reset. |

Os caminhos e linhas abaixo referem-se ao commit-base indicado. Prefixos usados na tabela: `VL = src/gps_truck_nav/nav_virtual_lanes/nav_virtual_lanes/`, `TB = src/gps_truck_nav/truck_bringup/`, `VP = src/gps_truck_nav/nav_virtual_lanes_planner/`.

## 2. O que o código realmente faz

| Componente e evidência | Constatação estática e consequência |
|---|---|
| `VL/lane_costmap_generator.py:17–33,85–94,132–154` | Publica três `OccupancyGrid`, com offsets 0/−5/+5 m e valores 20/60/100. Exterior começa em 0; bordas recebem 50. Portanto não há exterior lethal nesse gerador. Se estes valores fossem usados diretamente por `KeepoutFilter`, a faixa esquerda=100 seria lethal, e borda=50 seria custo intermediário. |
| Mesmo arquivo, `:65–77,136–146,166–200` | Interpolação usa 120 divisões por segmento, conserva yaw do início e preenche quadrados alinhados aos eixos ao redor dos pontos. Isso não equivale à rasterização de um corredor de largura constante. O teste de limite usa apenas o índice linear; é necessário validar coluna e linha separadamente para evitar passagem indevida entre linhas. |
| Mesmo arquivo, `:18–20,23–24,57,98–104,123–130,156–162` | Mensagens são publicadas uma vez, com QoS padrão volatile, esperas `fromLL` sem prazo, conexão artificial a `(0,0)` e quaternion de origem não inicializado para identidade. Um consumidor tardio pode não receber os grids. A origem presumida não é prova da pose inicial do caminhão. |
| `VL/lane_occupancy_detector.py:29–59` | Classifica células por intervalos fixos de **y global**. Não consulta centro/tangente da rota nem orientação da origem do grid. Contagem de células dividida apenas pela largura em células não é porcentagem de área ocupada de uma faixa. Considera somente valor 100, sem política explícita para unknown ou custos inflados. |
| `VL/lane_occupancy_detector.py:61–71`; `VL/visualize_lanes_from_gps_and_wps.py:39–55,88–96` | O detector reduz três escolhas a `Bool(preferred_lane == 1)`. O visualizador usa esse Bool como índice; `False` seleciona 0 e `True` seleciona 1. A escolha esquerda=2 se perde. Contador de estabilidade é atualizado, mas não condiciona a troca; não há histerese temporal efetiva. |
| `TB/scripts/gps_waypoint_logger.py:63–75,88–93`; `VL/visualize_lanes_from_gps_and_wps.py:70–74`; `VL/wall_costmap_generator.py:55–59`; `VL/lane_costmap_generator.py:42–44,173–174` | Logger grava yaw em radianos; dois visualizadores aplicam `radians()` ao YAML, enquanto o gerador de grids usa yaw diretamente em `sin/cos`. Há contratos incompatíveis. A unidade deve ser declarada e validada por arquivo, sem inferência silenciosa. |
| `VL/wall_costmap_generator.py:33–34,84,128–163`; `TB/params/truck_nav2_params.yaml:111–123,134–145,180–207` | O produtor publica paredes em `virtual_wall_points`. O YAML ativo escuta `/external_area_pointcloud` e `/sick_scan_2D`; referências a paredes virtuais estão comentadas. Publicar a nuvem ou o marker não prova que a costmap recebeu a restrição. Não foi identificado produtor de `/external_area_pointcloud` nos scripts ativos auditados. |
| `TB/params/truck_nav2_params.yaml:104–105,177–178,238–251,279–282` | `filters` está comentado; planner ativo é Smac. O plugin Virtual Lanes está comentado. As READMEs dos dois pacotes confirmam que não são iniciados pelo Quick Start. |
| `VP/src/virtual_lanes_planner.cpp:126–142,146–183,294–358` | Plugin usa pontos de um `Marker.LINE_STRIP`, seleciona trecho por distância e substitui início/fim. Não consulta custos de colisão nem restringe curvatura. Sem pontos, devolve reta start→goal. Em erro TF, preserva coordenadas e troca apenas `frame_id`. Ambos os comportamentos são inadequados como fallback de segurança. |
| Mesmo arquivo, `:82–90,154–156,166–178` | `lane_id` é lido na configuração; não há callback de atualização. Pontos substituem a posição de `msg.pose`, sem compor sua transformação completa. Não há rejeição explícita por idade da rota, tratamento completo de `Marker.DELETE` ou autorização de nova versão. |
| `TB/scripts/follow_dynamic_gps_wps_lanes.py:69–108,110–157,186–196` | Sender extrai posições de `MarkerArray`, força yaw=0, descarta pontos por `wp.x > robot_x` e publica um único Twist zero após pedido de cancelamento. Não comprova velocidade zero, conclusão efetiva do cancelamento, transição Ackermann ou domínio exclusivo de comandos. Assume navegação em +X. |
| `historical/nav_virtual_lanes_planner/src/virtual_lanes_planner.cpp:126–145,299–395`; `historical/COLCON_IGNORE:1` | Variante histórica força timestamps=0 e ajusta yaw do caminho, mas conserva fallback reto e relabeling após falha TF. Está fora do build suportado. Não substituí-la automaticamente pela atual. |

## 3. Restrições físicas e geometria

Preservar o footprint configurado de 9,605 × 3,478 m (`TB/params/truck_nav2_params.yaml:103,166`) e os raios mínimos de 11,6 m no Smac (`:251`) e MPPI (`:384`). Esses são valores do snapshot, sujeitos à conferência de frame e modelo no Gate 0; não são variáveis de otimização.

Usar uma única rota canônica em `map`, obtida de GPS por datum validado. Reamostrar por comprimento de arco, calcular tangente/normal coerentes e definir esquerda/direita em relação ao sentido crescente da rota. Identidade da rota deve incluir datum, frame, unidade angular, arquivo original e checksum. Rejeitar pontos não finitos, duplicatas degeneradas, yaw incompatível e segmentos cuja projeção seja ambígua. Em retornos e cruzamentos, a projeção deve conservar continuidade em comprimento de arco, sem saltar para outro trecho próximo.

Offsets históricos ±5 m e largura 5 m são evidência do código atual, não uma especificação validada. Larguras e offsets futuros devem ser parametrizados e congelados no protocolo da frente. Curvas deslocadas precisam de inspeção de auto-interseções e viabilidade; reduzir artificialmente o raio do veículo para acomodá-las não é aceitável.

O critério de contenção deve verificar o **polígono inteiro do caminhão e a região percorrida entre poses**, não somente `base_footprint` ou os pontos do path. Um veículo com 9,605 m pode invadir o exterior ao girar mesmo com seu centro dentro da faixa. Margens devem considerar resolução, erro de localização e discretização temporal, com valores e justificativa registrados antes dos ensaios.

## 4. Caminho nativo no Nav2 Iron

Foram consultados os arquivos oficiais no SHA `022e7e18570d75a969b96ccd1c3d5c5c443b3f12`, referenciado por `nav2_gps.repos:1–8`. Esse SHA é opção de source build; **não comprova equivalência ao pacote binário `ros-iron-navigation2` que será instalado**. A versão binária, plugin XML e comportamento devem ser confirmados no host.

Fontes consultadas:

- [CostmapFilter, conversão e worldToMask](https://github.com/ros-navigation/navigation2/blob/022e7e18570d75a969b96ccd1c3d5c5c443b3f12/nav2_costmap_2d/plugins/costmap_filters/costmap_filter.cpp).
- [KeepoutFilter, recepção e aplicação da máscara](https://github.com/ros-navigation/navigation2/blob/022e7e18570d75a969b96ccd1c3d5c5c443b3f12/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp).
- [LayeredCostmap, ordem de plugins e filters](https://github.com/ros-navigation/navigation2/blob/022e7e18570d75a969b96ccd1c3d5c5c443b3f12/nav2_costmap_2d/src/layered_costmap.cpp).

Conclusões limitadas a essa fonte:

1. `KeepoutFilter` aceita custos intermediários: `OccupancyGrid` 0…100 é convertido em `round(valor × 254/100)`. Assim, 100 significa lethal; custos menores podem representar preferência suave. Preferência não garante a faixa escolhida: a decisão ainda depende de obstáculos, custos e viabilidade Smac/MPPI.
2. `-1` se torna `NO_INFORMATION` e é ignorado pelo filtro. Ele não serve como exterior proibido. Fora da área coberta pela máscara também não há imposição do filtro. A máscara deve cobrir o domínio operacional e conter uma região externa explicitamente lethal; o supervisor deve detectar saída da cobertura.
3. O filtro preserva obstáculos existentes por combinação de máximo, exceto que células desconhecidas da costmap podem receber o custo válido da máscara. É necessário registrar e testar a política de unknown; uma região marcada permitida não prova que foi observada livre por sensores.
4. `CostmapFilterInfo` e máscara usam subscriptions `transient_local` e `reliable`. Produtor futuro deve fornecer QoS compatível, metadados consistentes e máscara íntegra antes de liberar movimento. Publicadores volatile atuais não satisfazem esse contrato.
5. Sem máscara ou sem transformação disponível, `KeepoutFilter::process()` registra mensagem e retorna. Isso **não bloqueia automaticamente a navegação**. `current_` ou lifecycle ativo isoladamente não comprovam aplicação da máscara. Deve existir verificação explícita de máscara, cobertura, TF e células efetivas em ambas as costmaps, além de parada controlada se a restrição deixar de ser válida.
6. `worldToMask()` usa posição/resolução e não aplica quaternion da origem. Rasterizar a geometria no frame escolhido e usar quaternion identidade na origem; não tentar orientar o raster apenas por `info.origin.orientation`.
7. `LayeredCostmap` processa os plugins primeiro e os filters depois. A `InflationLayer` normal não infla automaticamente obstáculos recém-introduzidos pelo filtro. Verificar margens pré-rasterizadas e collision checking; não assumir que `inflation_radius` do YAML resolve as bordas.
8. Não copiar parâmetros de documentação Rolling como `override_lethal_cost` e `lethal_override_cost`; eles não aparecem nessa implementação consultada.
9. Filtros podem ser configurados separadamente nas costmaps global e local. Neste snapshot, elas usam `map` e `odom`, respectivamente (`TB/params/truck_nav2_params.yaml:94,162`): transformação correta e recente é requisito, e a mesma máscara física precisa produzir limites consistentes nas duas.

Proposta inicial: uma máscara única de corredores permitidos com exterior lethal e preferência central por custo suave, publicada junto com `CostmapFilterInfo` adequado. Manter obstáculos sensoriais separados. Só adicionar seleção discreta de faixa se ensaios mostrarem que custos suaves não fornecem o comportamento necessário. Não criar outro global planner nesta fase.

## 5. Sequência de desenvolvimento futura

| Etapa | Entrega e condição para avançar |
|---|---|
| VL0 — contrato | Congelar frame/datum/unidades, geometria, cobertura, footprint, tolerâncias e convenção de lados. Registrar rastreabilidade do YAML/modelo. |
| VL1 — geometria offline | Biblioteca pura de rota, corredores e raster. Testes com reta, curva, trajetória rotacionada/transladada, U-turn, cruzamento e pontos degenerados. Confirmar que a saída varia apenas pela transformação esperada. |
| VL2 — costmaps | Ativar filtros em configuração própria, sem alterar o baseline. Registrar máscara, info, parâmetros e amostras de células global/local. Verificar exterior lethal, custos internos, preservação de obstáculos e comportamento após limpeza/reset. |
| VL3 — navegação confinada | Executar Smac–MPPI em reta e curvas. Verificar footprint/swept area, raio e plano executado. RViz serve somente como diagnóstico. |
| VL4 — bloqueios e transições | Introduzir bloqueio central, alternativas de ambos os lados, todas bloqueadas e remoção do obstáculo. Demonstrar mudança viável e retorno ao centro com histerese. |
| VL5 — falhas e repetibilidade | Testar máscara ausente, obsoleta ou fora de cobertura, perda TF, publisher reiniciado, frame errado e reset entre execuções. Repetir cenários e apresentar denominadores completos. |

Caso seja necessário seletor discreto, usar estado explícito com `lane_id ∈ {right, center, left, none}`, timestamp, versão de rota, progresso e motivo. `none` significa nenhuma alternativa validada. Definir estados de manutenção de faixa, avaliação, transição, estabilização e parada; não usar Bool como escolha ternária. A preferência central só pode provocar retorno após disponibilidade sustentada, transição viável e conclusão/coordenação da ação vigente. Limiares de histerese devem ser fixados antes da validação.

## 6. Matriz mínima de aceitação

Todos os itens abaixo estão **PROPOSED BUT NOT TESTED**. Tolerâncias métricas e limites de tempo devem ser fixados com a resolução/clock do ambiente antes de executar a matriz.

| Ensaio | Evidência exigida |
|---|---|
| Reta com três faixas livres | Centro preferido; máscara efetiva nas duas costmaps; trajetória e footprint dentro do permitido. |
| Mesmo cenário rotacionado/transladado | Decisões esquerda/direita e ocupação equivalentes; nenhum uso implícito de +X ou y global. |
| Curva próxima ao limite cinemático | Caminho e execução respeitam 11,6 m; nenhuma invasão pelo balanço dianteiro/traseiro. |
| Centro bloqueado, direita livre | Transição contínua e collision-checked; obstáculos não são removidos pela máscara. |
| Centro e direita bloqueados, esquerda livre | Esquerda realmente selecionável e executável; prova do contrato de três estados. |
| Todas as faixas bloqueadas | Sem atalho pelo exterior; parada/controlador sem progresso classificados separadamente de falha de infraestrutura. |
| Obstáculo removido | Retorno ao centro somente após critério estável e transição admissível; contar alternâncias e cancelamentos. |
| Obstáculo junto à borda e faixa estreita | Rejeição de passagem quando o footprint completo não cabe, mesmo que o ponto central passe. |
| Máscara/TF ausente ou inválida | Missão não liberada, ou parada controlada durante execução; sem fallback reto. |
| Limite da máscara e U-turn | Sem escape por região não coberta; progresso da rota não salta para ramo vizinho. |
| Clear costmap e reinício | Keepout continua efetivo após recuperação; nenhuma máscara/rota de versão anterior é aceita. |

Por execução, guardar rota e máscara originais/hashes, pacote e versão do filtro, parâmetros carregados, TF, poses/footprints, planos, comandos, estados e motivos de seleção, ação Nav2, costmaps e eventos de reset. Calcular violações de contenção, distância à borda, curvatura, duração de bloqueios, transições, alternâncias, conclusão e falhas técnicas com seus respectivos denominadores. Contato físico só pode ser classificado com uma fonte de ground truth definida e validada; distância à costmap não é a mesma medida.

## 7. Próxima decisão de revisão

Primeiro concluir e revisar os Gates 0/1 do auto-tuning sem Virtual Lanes. O início de VL0/VL1 pode reutilizar a infraestrutura validada depois, sem bloquear o ciclo básico. A aceitação desta frente exige novas execuções, protocolo próprio e revisão independente. Nenhuma afirmação sobre Virtual Lanes pode ser atribuída retroativamente ao benchmark histórico.
