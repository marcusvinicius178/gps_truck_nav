# Velocidades históricas e perfis prospectivos — revisão 3

**Não há evidência para trocar 25.0 por 2.5 como correção de digitação.** O histórico Git contém uma sequência de valores altos em controladores distintos, e o serviço ROS da imagem a21403f confirma `FollowPath.vx_max=25.0`. O que está demonstrado é o valor declarado/aplicado ao nó. Atingir 90 km/h, consumir esse máximo numa trajetória ou explicar a intenção original do autor não está demonstrado.

Gate 0 e Gate 1 permanecem NOT_YET_VERIFIED. Os novos perfis são completos, separados do YAML histórico e autorizados somente para diagnóstico sem goal. Converter as unidades corretamente e igualar envelopes de parâmetros não valida a planta nem libera movimento.

## 1. Procedência dos números

Foram inspecionados o relatório independente a21403f integral, os JSONs estruturados completos de parâmetros/observações, os dumps de planner/controller/smoother e a história Git dos YAMLs. Valores da tabela são obtidos por parsing YAML, portanto distinguem configuração ativa de comentários. As datas são de commit, não datas de experimentos executados.

| Commit | Data do commit | Configuração ativa no arquivo | Smoother longitudinal |
|---|---|---|---|
| `43f74dc88c6f802344ada4470d34b1756f857efd` | 02/09/2025 | MPPI `vx_max=10.0` no caminho antigo `src/truck_bringup/params/truck_nav2_params.yaml` | ±8.33 m/s |
| `e56218bc57f9157c2b688884dddad8c443300d79` | 15/10/2025 | MPPI `vx_max=10.0`, `wz_max=1.5`, horizonte 60×0.05=3 s, no caminho `src/gps_truck_nav/truck_bringup/params/truck_nav2_params.yaml` | ±8.33 m/s |
| `366e488acb511b76c9573951d43de03518a68f5a` | 28/10/2025 | MPPI `vx_max=10.0` | ±8.33 m/s |
| `212503e151dafd651c278748a70d1b825bc4b938` | 28/11/2025 | MPPI `vx_max=10.0` | ±8.33 m/s |
| `4746785cbd6835b446ad84b71c7077dc74356fd9` | 28/11/2025 | MPPI `vx_max=20.0` | ±8.33 m/s |
| `422a16776847358387131c73e116ba7d699eb4da` | 24/12/2025 | RPP `desired_linear_vel=22.0`; o bloco MPPI com `vx_max=25.0` está comentado | ±8.33 m/s |
| `2eb40fd44b48baa31b31dcd1047ab89f8a128c3b` | 09/01/2026 | Reativa MPPI `vx_max=25.0`, `wz_max=0.4`, horizonte 40×0.05=2 s; RPP 22 passa a comentário | ±8.33 m/s |
| `00747641ffe04ceb484fc0bb813babb12e4f87e5` até a21403f | 23/09/2026 em diante | Preserva MPPI 25 na organização canônica | ±8.33 m/s |

O snapshot arquivado `historical/root_snapshot/params/truck_nav2_params.yaml` também contém MPPI 10; SHA-256 `728a1341b6fd4f82fe3143269e6b05b6b464fa65992eed4a7cb45f70ef051eff`. O YAML canônico atual tem SHA-256 `722169c640ac8a37c10ca1097ff212366bac7ae58ac2f596608bfc7a9454eef7`. Ambos permanecem intactos. A busca nas revisões percorridas não encontrou `vx_max=2.5` ativo. Não extrapolar essa ausência para arquivos privados ou revisões não disponíveis.

Comentários mencionando 5.0/8.5 m/s não sobrepõem o valor YAML ativo. O texto `default_controller: RegulatedPurePursuit` no BT tampouco prova execução de RPP: o plugin carregado e a lista efetiva do controller são a evidência adequada. Títulos de commits que dizem que algo funcionou, ou que não se moveu, são pistas de desenvolvimento; não são registros de aprovação de missão. As alterações históricas também modificam vários outros parâmetros, portanto não constituem comparação controlada de velocidade isolada.

## 2. Semântica e limites observados

Na fonte Nav2 **1.2.10**, `vx_max/vx_min` são limites de velocidade longitudinal em m/s e `wz_max` é limite de yaw rate em rad/s. O horizonte do MPPI é formado por `time_steps × model_dt`. O valor RPP `desired_linear_vel` é o máximo desejado antes das regulações de curvatura/obstáculo/aproximação, não uma promessa de velocidade realizada.

| Ponto da cadeia a21403f | Evidência | Interpretação limitada |
|---|---|---|
| MPPI | Get/Describe tipado: `vx_max=25`, `vx_min=-2.25`, `wz_max=0.4`, modelo Ackermann, 40×0.05 s | Envelope do controlador; 25 m/s equivale a 90 km/h. Não é um erro de unidade convertido automaticamente pelo Nav2. |
| Velocity smoother | Dump efetivo: `max_velocity=[8.33,0,0.72]`, `min_velocity=[-8.33,0,-0.72]`, OPEN_LOOP, `scale_velocities=false`, 20 Hz | O ramo que atravessa esse nó limita x a aproximadamente 29.988 km/h e suaviza componentes; não converte yaw rate em direção. |
| Gazebo Ackermann | SDF preservado: `max_speed=20`, `max_steer=0.6458`; semântica upstream inspecionada | 20 m/s é um teto XML do plugin, não o limite único da cadeia nem prova de velocidade atingida. A identidade do binário carregado precisa ser correlacionada. |
| Behavior server | Quatro publishers em `/cmd_vel`, além do publisher do smoother, no grafo recebido | Há caminhos que contornam o smoother. Não é correto afirmar que toda atuação está limitada a 8.33 m/s por esse nó. |

O lançamento, tanto composto como não composto, remapeia controller para `/cmd_vel_nav` e smoother para entrada `/cmd_vel_nav` / saída `/cmd_vel`. Os behaviors conservam `/cmd_vel`; o assinante observado nesse tópico é `ackermann_drive`. As BTs sem ações de recuperação de movimento reduzem o uso desses caminhos, mas não removem seus servidores nem garantem autoridade exclusiva. Um adaptador final precisa receber todos os produtores por arbitragem/roteamento explícitos; proteger apenas o ramo MPPI deixa bypass.

Na implementação 1.2.10 do smoother, primeiro há clamp **por componente** aos limites absolutos. Depois `scale_velocities`, quando habilitado, influencia o vetor de variações usado para limites de aceleração. Portanto ligar essa flag não corrige automaticamente curvatura alterada pelo clamp absoluto, não converte unidades e não substitui um contrato conjunto final `(v,ω)`. Essa flag fica preservada nesta revisão.

O envelope atual prevê até 50 m de deslocamento retilíneo em dois segundos a 25 m/s, enquanto o teto de 8.33 do smoother corresponderia a 16.66 m nesse intervalo; `prune_distance=15` permanece na configuração. Esses cálculos dimensionais evidenciam a necessidade de reconciliar previsão/execução e validar o caminho disponível. Não demonstram que o MPPI usou o máximo, que uma trajetória é inválida nem que o caminhão atingiu essas velocidades.

## 3. Perfis 20 e 25 km/h: proposta explícita e reversível

`config/auto_tuning/navigation_profiles/20kmh.yaml` e `25kmh.yaml` são YAMLs completos gerados do snapshot canônico. O manifest fixa fonte, hashes, deltas, valores de readback e pendências. `prepare_navigation_profiles.py` recusa fonte de hash diferente e recomputa os arquivos esperados ao verificá-los, inclusive o manifest. Não basta alterar o hash no manifest para fazer uma edição indevida passar.

| Valor | 20kmh | 25kmh | Fundamentação |
|---|---:|---:|---|
| MPPI `vx_max` e smoother `max_velocity[0]` | 50/9 ≈ 5.555556 m/s | 125/18 ≈ 6.944444 m/s | Conversão SI exata dos tetos experimentais solicitados. |
| MPPI `vx_min` e smoother `min_velocity[0]` | −2.25 m/s | −2.25 m/s | MPPI histórico preservado; smoother reconciliado ao mesmo envelope. Não é velocidade autorizada de ensaio em ré. |
| MPPI `wz_max`; smoother limites yaw | 0.4; ±0.4 rad/s | 0.4; ±0.4 rad/s | Reutiliza o teto angular MPPI, sem inventar capacidade mecânica. |
| Raio Smac e MPPI | 11.6 m, pendente | 11.6 m, pendente | Preservado para tornar o conflito visível, não aprovado nem substituído por 13.09. |
| Horizonte MPPI | 2 s / 11.111111 m retos no teto | 2 s / 13.888889 m retos no teto | Tempo preservado; distância apenas cinemática. |
| Raio `v/ω` nos dois tetos simultâneos | 13.888889 m | 17.361111 m | Não é raio mínimo da planta; curvas mais fechadas exigem reduzir v ou outro contrato viável. |

Há somente cinco folhas alteradas por perfil: `FollowPath.vx_max` e índices 0/2 de `velocity_smoother.max_velocity/min_velocity`. Pesos/críticos, acelerações, timeout, costmaps, BT, modelos de movimento e demais valores permanecem iguais. A ré continua disponível na configuração, mas seu ensaio depende do protocolo próprio; `DUBIN` no planner não é convertido para REEDS_SHEPP apenas por haver `vx_min<0` no controlador.

A justificativa dos novos tetos é o protocolo experimental solicitado, **não** uma especificação de segurança do Arocs. Os limites de aceleração/deceleração herdados ±3 m/s² e ±1 rad/s² também não foram fisicamente validados. Os ensaios de atuação devem começar nos valores mínimos definidos e aprovados pelo protocolo; 20 km/h é uma etapa posterior à calibração básica, e 25 km/h depende do resultado de 20 km/h. Nenhum dado da plataforma física nem dos 212 experimentos é reclassificado por esses perfis.

No topo da velocidade, 20/25 km/h não podem ser exigidos em qualquer curva. Com `wz_max=0.4`, os raios acima e acelerações laterais cinemáticas correspondentes (aproximadamente 2.22/2.78 m/s²) não foram aprovados como limites de estabilidade. Baixar `vx_max` tampouco elimina pedidos de raio 11.6 em velocidades menores. A diferença para o bound ideal estático ~13.09 m, a referência base/eixo traseiro, saturações dos joints e a resposta dinâmica permanecem `BLOCKED_ACTUATION_REVIEW`.

## 4. Verificação e comparação científica

É permitido carregar um perfil no **probe sem goal** para demonstrar os valores efetivos. Essa etapa não conecta o novo núcleo de conversão à planta e não autoriza Twist. O helper conserva `canonical` como default e registra a seleção explícita. O coletor deve comparar o YAML selecionado, consultar os dois plugins **e** os vetores do velocity_smoother, registrar arquivo/hash, confirmar o grafo e manter `mission_ready=false`. Um PASS de parâmetros deve ser descrito como readback, não como atingimento da velocidade experimental.

API para a integração: `load_selected_profile(canonical_path, profiles_directory, '20kmh'|'25kmh')` retorna `(configuração completa, entrada do manifest)` após verificar todos os bytes; `expected_runtime_parameters(config)` retorna o contrato por nó/nome. O manifest contém `profiles[id].expected_parameters`, `profiles[id].changes` e `profiles[id].sha256`. A CLI `verify` somente lê/verifica arquivos; geração usa um diretório novo e nunca sobrescreve saída anterior.

O runbook principal contém os comandos completos do build e do probe com `--navigation-profile`. O estado esperado dos assets é `PREPARED_DIAGNOSTIC_ONLY`, com `readiness=BLOCKED_ACTUATION_REVIEW`; o status da captura runtime depende da execução real. Todos os campos de autorização permanecem falsos nos manifests.

Para comparação A/B futura, separar dois fatores:

1. Preservar fonte/imagem/evidência histórica e seu contrato real. Uma navegação relatada como funcional não pode ser reconstruída somente a partir de um YAML público; registrar lacunas de versão, mapa, planta, interface, sensores, sementes e execução. Não executar a cadeia histórica de atuação neste estágio.
2. Depois de validar a planta/interface prospectiva, congelá-las para comparar os perfis e, mais tarde, hiperparâmetros. Mesma planta, geometria, percepção, rota, estado inicial, seeds quando controláveis e reset comprovado. Não atribuir ao tuning um ganho obtido ao corrigir a interface yaw-rate/esterçamento.

Critérios mínimos antes de movimento: readbacks corretos com proveniência, atuação exclusiva validada, referências geométricas e limites reconciliados, watchdog final demonstrado, cancelamento/parada diferenciados, percepção/localização aprovadas e autorização do revisor. Somente um protocolo supervisionado autorizado pode avaliar velocidade atingida, trajetórias, saturações e segurança funcional.

## Fontes primárias e rastreabilidade

- História local de `marcusvinicius178/gps_truck_nav`, commits completos da tabela, caminhos dos dois YAMLs preservados. Não houve alteração nesses arquivos.
- ZIP `nav2_phase_a_second_review_evidence.zip`, SHA-256 `e2ffc8d0c875abc816cbccb5470ff25902e430691aba75e51cafb4c14445221d`.
- `probe/runtime_probe/plugin_parameters.json`, SHA-256 `7606f921f5a7e8fa1061a980a484a6f48bacfbcb24e09dcfbdd8a45cfb3de1d5`; readbacks obrigatórios completos.
- `probe/runtime_probe/readiness_observations.json`, SHA-256 `0625c8a788f8a0641398eaed416683c45f4784020cbd4b1e74af8a6372f2cbfb`; grafo de publishers/subscribers.
- [MPPI README 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/README.md), blob `f8a90f1cbc00843b4bb013847cc5437b3a445967`: unidades, horizonte e visualização.
- [Velocity smoother 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_velocity_smoother/src/velocity_smoother.cpp), blob `d3dd5c2a9979c5863d4106784867d81bd92662cf`: clamp absoluto, feedback, timeout e limites de variação.
- [RPP README 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_regulated_pure_pursuit_controller/README.md), blob `fc1d738999308863c1b28a44a3d99f9ee05fa6c1`: máximo desejado e regulações. Não comprova a versão binária de uma execução histórica.
- `ackermann_interface_contract.md` e `actuation_validation_protocol.md`: semântica do plugin, geometria e sequência de validação pendente.
