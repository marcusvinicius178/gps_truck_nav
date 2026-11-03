# Protocolo prospectivo de atuação e demonstração supervisionada

**Estado: PREPARADO, NÃO EXECUTADO. Gate 0 e Gate 1 NOT_YET_VERIFIED.** A autorização atual cobre preparação, testes offline, probe e visualização de observação. Não cobre envio de goal, Twist, comando de junta, teste dinâmico ou conexão do conversor à planta. Este documento define as condições para uma autorização posterior; não contém comando acionável de movimento.

## Evidência e objetivo funcional

A terceira revisão leu integralmente o relatório `nav2_revisao_independente_a21403f.md`, a geometria e o grafo estruturados do ZIP `nav2_phase_a_second_review_evidence.zip` (SHA-256 `e2ffc8d0c875abc816cbccb5470ff25902e430691aba75e51cafb4c14445221d`). O runtime confirmou TF, sensores, costmaps, BTs e parâmetros obrigatórios. Preservar esses reparos; avisos cosméticos não são objetivo de engenharia.

O grafo recebido mostra **cinco publishers em `/cmd_vel`**: quatro endpoints do behavior_server e um do velocity_smoother. O Ackermann assina esse tópico; o controller publica `/cmd_vel_nav`, assinado pelo smoother. Portanto não existe hoje autoridade final exclusiva que possa garantir conversão de todas as solicitações. O limite 8,33 m/s do smoother não limita automaticamente o caminho dos behaviors. A interface física do caminhão e o runtime histórico privado não foram demonstrados como idênticos a esse plugin Gazebo.

O resultado desejado é observável: um veículo corretamente representado no mundo, um objetivo explicitamente escolhido, um caminho inspecionável, execução limitada que pode ser cancelada e interrompida, parada medida e logs suficientes para reproduzir o ensaio. Um relatório sem warnings ou com mais PASS não substitui esse resultado.

## Núcleo e adaptador de observação: limite da entrega

`tools/auto_tuning/actuation_contract.py` contém matemática e uma máquina de estados **offline**. Não importa ROS, não cria publisher, não abre socket e não é chamado pelo bringup. Todas as decisões retornam `runtime_authorized=false` e `physical_stop_verified=false`, inclusive no estado de modelo ARMED. Os testes usam assertions artificiais; preenchê-las com `true` não constitui aprovação real.

O incremento de interface prepara `tools/auto_tuning/ackermann_adapter.py` como observador ROS 2: recebe comandos para análise e registra resultados candidatos, **sem criar publisher de atuação** e sem ligar a conversão ao caminhão. Essa etapa serve para verificar integração e unidades no runtime; não prova seguimento de comandos nem oferece parada operacional. A escolha é manter temporariamente o Gazebo Classic 11 e usar uma adaptação ROS 2 pequena, preservando o plugin upstream. A decisão e a validação mínima estão em `prospective_plant_validation.md`.

Esse observador assina o tópico de entrada, `/clock` e `/odom`, sem clientes de serviço, publisher ou opção live. Guarda os seis componentes originais do Twist, GID e recepção monotônica, além do cálculo condicionado. `SHADOW_COMPLETE` significa somente que a captura limitada terminou; continua `BLOCKED_ACTUATION_REVIEW`. Na observação sem goal, ausência de comando produz `NOT_EXERCISED`, resultado esperado, não falha a corrigir com um comando de teste. Um comando não nulo inesperado interrompe o observador e deve encerrar a tentativa pelo supervisor próprio; o observador não envia um zero nem para a planta por si só. Os thresholds da política shadow classificam evidência, não validam frenagem.


| API | Contrato |
|---|---|
| `Geometry.from_static_audit(audit)` | Copia geometria/limites e SHA do modelo auditado; não converte geometria estática em geometria runtime validada. |
| `Policy(...)` | Exige valores explícitos para limites, banda de parada/histerese e deadlines monotônicos. Não fornece defaults operacionais. |
| `convert(v, omega, geometry, policy)` | Recebe m/s e rad/s no ponto da bicicleta definido; devolve candidato de velocidade m/s e campo angular do plugin em rad. Nenhum objeto Twist é criado. |
| `OfflineSupervisor.arm/submit/poll/inhibit` | Modela autorização explícita, identidade fixada, autoridade única, frescor, relógios, watchdog, reversão e falha latched; não é o watchdog operacional. |

A inversa de direção parte da referência no eixo traseiro: `delta=atan(L*omega/v_R)` para `v_R≠0`. A velocidade do comando também precisa corresponder ao PID do plugin: o upstream compara o alvo com a velocidade periférica da roda traseira direita, portanto `v_plugin=v_R−omega*y_right`, onde `y_right` é a coordenada lateral dessa roda relativa ao centro do eixo. O campo angular enviado ao plugin deve satisfazer `u=sign(v_plugin)*delta`, pois o plugin multiplica esse campo pelo sinal do seu comando linear uma única vez. Dentro do envelope geométrico considerado, o sinal de `v_plugin` coincide com o de `v_R`; essa condição deve ser verificada e não presumida para geometria arbitrária.

Usar `v_R` diretamente no PID de uma única roda em curva introduziria erro de velocidade mesmo com a conversão angular correta. Os testes devem reconstruir ambos os campos, nos quatro quadrantes, e verificar zero, entradas inválidas e ambos os joints. Curvatura acima do envelope estático é rejeitada; não há clipping de esterçamento para mascará-la. Se a política escolher redução de velocidade, v e omega são reduzidos pelo mesmo fator, preservando curvatura. Alterar somente v poderia aumentar a curvatura solicitada.

O modelo exige parada solicitada e standstill medido antes de mudar o sinal de deslocamento. Aceitar um novo pacote depois do deadline não rearma um watchdog já expirado. Dados vencidos, NaN/Inf, relógio regressivo/parado, publisher extra, source GID diferente, identidade alterada ou overspeed medido inibem a saída candidata e exigem rearmamento explícito. As falhas preservam seu motivo. Nenhum desses testes demonstra desaceleração, aderência, estabilidade, tempo de stop real ou cobertura de morte do próprio processo.

As regressões do núcleo incluem uma varredura de curvaturas realizáveis nos dois sentidos, reconstrução da taxa angular pelas equações do plugin, saturação conjunta, deadlines exatos, identidade alterada, bypass e stop/reversão. A confirmação de standstill é renovada com a velocidade observada atual: uma parada passada não permite ré após o veículo voltar a rolar. No checkout dedicado, este comando completo executa somente esses testes offline:

```bash
cd "$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06/gps_truck_nav"
/usr/bin/python3 -I -m unittest discover -s tests -p test_actuation_contract.py -v
```

Esses testes importam apenas stdlib e os arquivos do repositório. Os thresholds 0,05/0,10 m/s e tempos presentes nas fixtures são **valores sintéticos para exercitar ramos**, não recomendações de operação. A implementação futura precisa de limites de aceleração/desaceleração, slew de direção e margens físicas fundamentados; esse comportamento dinâmico não foi implementado no núcleo estático.

## Geometria, origem cinemática e velocidade

O cálculo estático reproduz L=6,816500073 m, W=2,598120190 m, raio traseiro 0,536 m. `max_steer=0,6458` supera o envelope dos dois joints ±0,524; a expressão conservadora resulta em ângulo virtual 0,479957 rad e raio **no eixo traseiro ideal** 13,094652 m. Isso não é uma medição de curva do Gazebo nem o raio externo da carroceria.

`base_footprint` fica 3,502 m à frente do eixo traseiro selecionado. Para deslocamento planar `r=(a,b)` entre esse eixo e a base, `v_Bx=v_R−omega*b` e `v_By=omega*a`. Com b≈0, v longitudinal coincide, mas a base tem componente lateral em curva. O MPPI Ackermann usa uma referência não holonômica; converter rad/s em rad não resolve sozinho essa diferença. Para b=0 e curva ideal, `R_base=sqrt(R_rear²+a²)`; footprint e envelope varrido ainda ampliam a região ocupada.

O candidato prospectivo usa um frame adicional `rear_axle_nav`, fixo 3,502 m atrás de `base_footprint`, para expressar o ponto da bicicleta. A geometria histórica não é deslocada. O footprint no novo frame deve representar a mesma carroceria, e transforms, pose/velocidade de referência e métricas devem declarar essa origem. O frame e a configuração candidatos precisam de confirmação no runtime; a existência dos arquivos não comprova alinhamento aplicado. O footprint traduzido +3,502 m em x mantém o mesmo polígono da carroceria no mundo. Os goals prospectivos passam a declarar a pose desejada do eixo traseiro, com `SimpleGoalChecker`; não aplicar silenciosamente o offset frontal de 5 m do goal checker histórico. A referência das métricas também deve ser explícita. Perfis anteriores e YAML histórico permanecem rastreáveis.

Antes de decidir um raio prospectivo, medir e registrar no modelo carregado as colisões/joints selecionados, transforms, limites e origem; correlacionar essa evidência com SHA/modelo, biblioteca mapeada e versão. As APIs de leitura existentes no ZIP não incluem GetEntityState/GetModelState. Não presumir esses serviços: identificar primeiro um recurso de leitura existente do Gazebo transport que exponha pose/juntas do modelo carregado e registrar seu referencial. Somente se a leitura disponível for insuficiente, propor uma observação adicional mínima, separada e versionada, sem alterar a dinâmica. Não desenvolver um novo plugin de atuação para preencher essa lacuna de medição. TF/odometria/GNSS derivados uns dos outros não constituem ground truth independente.

20 km/h = **50/9 m/s** e 25 km/h = **125/18 m/s**. São envelopes candidatos futuros; não são velocidade garantida ao longo de uma curva nem autorização para começar o primeiro ensaio a essa velocidade. O valor MPPI 25,0 m/s não foi reinterpretado como 2,5. Os perfis da revisão anterior conservam o raio histórico 11,6 m como evidência; o novo candidato usa **13,10 m no eixo traseiro**, arredondamento para cima do bound estático 13,094652 m, e limite virtual `atan(L/13.10)`. Esses valores resolvem a incompatibilidade matemática com ±0,524 rad, mas não são margem dinâmica de segurança nem raio medido. Rodas intermediárias fixas e a assimetria dos centros frontais permanecem no modelo; contato, arrasto e eventual deslizamento precisam ser observados no comissionamento e não são corrigidos por essa equação ideal.

Os perfis de navegação candidatos 20/25 km/h são inicialmente somente à frente, com MPPI e smoother reconciliados e origem `rear_axle_nav`. A conversão continua testada para ré; comissionar ré é um teste separado antes de permitir um perfil que a utilize. Ler os valores efetivos de planner, controller e smoother, incluindo raio/modelo de movimento, é necessário antes de conectá-los à planta. A dinâmica ainda não está aprovada.

Velocidade exige também distância de parada e aceleração lateral. Para uma curva ideal, `a_y=v²/R` e uma estimativa cinemática de distância de parada inclui `v*t_latency+v²/(2*a_brake)`. Esses cálculos não calibram `a_brake`, latência, estabilidade, contato/pneus ou margens. Não escolher números para obter PASS: especificar e aprovar limites a partir de evidência antes de aplicar um envelope 20 ou 25 km/h.

## Arquitetura que precisa existir antes do primeiro movimento

Todos os produtores de navegação, inclusive behaviors, devem passar por arbitragem documentada ainda em unidades v/omega. O smoother permanece antes da conversão. O destino do plugin deve assinar um tópico exclusivo, publicado apenas pelo estágio final aprovado; nenhum tópico legado pode contornar esse estágio. GIDs e endpoints devem sustentar essa verificação em runtime, com observação renovada e detecção de perda/duplicação da autoridade.

O estágio final precisa de expiração e contenção independentes da continuidade do controller/smoother/conversor e do avanço do clock simulado. No upstream inspecionado, o callback Ackermann mantém o último alvo e não implementa timeout. Uma publicação zero emitida pelo conversor antes de morrer não resolve o caso em que ele morre antes de publicar. A futura integração deve combinar watchdog do comando, autoridade única e supervisor externo da simulação, capaz de encerrar somente a tentativa própria se o estágio final morrer ou a observabilidade falhar. Essa contenção deve ser testada por injeção de falhas e registrada por versão; não editar `/opt/ros/iron` nem afirmar que o core/shadow adapter já a implementa. Encerrar a simulação não demonstra frenagem física.

O candidato reserva um tópico privado de atuação, sem publishers na etapa de observação, para impedir que a cadeia legada alcance diretamente o plugin. Isso prepara a separação; não resolve sozinho a arbitragem futura dos produtores nem constitui autorização de publicação.

Arming operacional exigirá um registro de aprovação que identifique geometria carregada, referência, limites, unidade/semântica em ré, identidade de biblioteca, roteamento, proteção no destino e escopo de autorização. Falha exige inibição latched e nova decisão explícita. A revisão dessa arquitetura precede sua conexão; nenhum arquivo desta entrega liga a conversão ao Arocs.

## Sequência de ensaios para autorização posterior

Cada linha é uma etapa futura com condições de entrada próprias. Falhar em uma etapa impede avançar para a seguinte. Não executar a tabela como uma lista de comandos autorizados.

| Etapa | Escopo limitado | Evidência de saída |
|---|---|---|
| O — observação atual | Abrir Gazebo/RViz no modo de observação, sem ferramentas de goal/pose/teleop, sem converter ou produzir comandos. | Arocs visível, TF único, sensores, footprint, duas costmaps e relógio avançando. Trajetórias só aparecem se previamente existentes ou produzidas por uma consulta futura autorizada. |
| S — integração estática | Em ambiente isolado, validar adaptador e proteção no destino com sink artificial, sem planta conectada. Injetar quatro quadrantes, zero, saturações, perda de produtor/consumidor, processo morto, clock parado, origem errada e publisher concorrente. | Campos solicitados/aplicados, deadlines, falhas latched, remoção de bypass e prova de expiração no destino. Testes offline desta entrega são parte de S, não conclusão dela. |
| D0 — comissionamento imóvel | Após aprovação da arquitetura e autorização dinâmica delimitada, medir geometria carregada e proteção final. Nenhuma missão. | Identidade e referência reconciliadas; zeragem/inibição sem deslocamento indevido; parada confirmada por ground truth independente. |
| D1 — calibração mínima | Somente simulação isolada, área livre delimitada e sem rota histórica. Ensaios curtos e separados de reta, curvaturas pequenas e ré, em velocidade baixa aprovada a partir da resolução e distância disponível. | v/omega pedido, u do plugin, targets de ambas as rodas, ângulos medidos, v/omega medido, pose/yaw ground truth, saturações e resíduos. Raio por referência declarado; nenhum limite novo baseado só no SDF. |
| D2 — interrupções | Repetir caso mínimo com cancelamento, parada solicitada, perda do controller/smoother/adaptador, publisher concorrente e clock parado. Limites/stop pré-aprovados. | Cancel ack separado de velocidade zero; latência e distância de parada, expiração do destino, estado final e ausência de rearmamento automático. |
| G0 — objetivo supervisionado | Após D0–D2 aprovados, selecionar um objetivo em área/corredor predefinido, inspecionar path e footprint/envelope antes de autorizar execução. Uma missão de duração/distância limitadas, operador presente. | Objetivo/frame/stamp congelados, path, feedback/action result, trajetória medida, métricas, interrupções e cleanup/reset. A velocidade aumenta por patamares autorizados até o envelope de 20 km/h, mantendo limites de curva/parada. |
| V25 — extensão | Depois de repetibilidade a 20 km/h e revisão de distâncias, estabilidade, percepção e frenagem, autorizar separadamente o envelope 25 km/h sobre a mesma planta. | Mesmo protocolo e cenário, velocidade efetiva medida, limites registrados e comparação sem misturar mudanças de planta/interface com tuning. |
| G1 — automação | Somente após Gate 0 aprovado, dois trials com reset comprovado e mesmos artefatos. | Automação, estado inicial comparável, métricas íntegra e falhas observáveis. Optuna e held-out vêm depois. |

Na etapa G0, escolher o objetivo não deve iniciar automaticamente o caminhão. Separar **seleção**, **planejamento/inspeção** e **execução autorizada**. O operador deve ver Arocs/footprint, goal, path, scan, costmaps, estado action, velocidades solicitada/aplicada/medida, estado de inibição e saturações. Interface de observação atual mantém ferramentas interativas desabilitadas; a versão que habilitar seleção/execução precisa de revisão própria.

## Cancelamento, parada e encerramento

Cancelamento Nav2 solicita término da ação; sua aceitação não comprova que o veículo já parou. A interrupção operacional deve primeiro inibir a cadeia final pelo mecanismo independente aprovado, solicitar cancelamento e acompanhar velocidade/pose até standstill confirmado pelo critério e duração predefinidos. Não depender apenas de fechar RViz, matar um controller ou enviar um único zero.

Se houver perda de observabilidade, falha de expiração ou deslocamento fora do envelope aprovado, interromper o ensaio pelo supervisor da simulação e parar somente o container daquela tentativa, por ID e labels registrados. Parar um container encerra aquela simulação; não é prova de frenagem do veículo, não aprova o caso e não comprova reset. Preservar todos os logs e marcar o ensaio interrompido/inválido. Não usar comandos globais de Docker, matar processos de outros ambientes ou reiniciar o daemon.

## Registro mínimo e comparação científica

Antes de autorizar a etapa dinâmica, preencher e versionar um registro com: commit/tree, image ID/digest, hashes de YAML/SDF/URDF/plugins/BTs, perfil, hipótese, cenário/world/datum, referência cinemática, área permitida, goal e envelope da rota, direção permitida, velocidade/curvatura/steering/accel/stop aprovados, tempo/distância máximos, limiares de abortagem, operador, autorização, mecanismos de parada e procedimento de reset. Campos pendentes bloqueiam a execução; a tabela acima não fornece defaults para substituí-los.

Registrar séries temporalmente alinhadas de clock simulado e recepção monotônica, v/omega original e suavizado, candidato/aplicado de esterçamento, targets/posições dos joints, velocidade/yaw rate e pose independentes, sensores/TF/costmaps/path/action, motivo/frequência/duração das saturações, estado do watchdog e logs de cancel/stop. Preservar mensagens originais e unidades. Retornos de API e logs de configuração são evidência distinta de execução da dinâmica.

O histórico é referência documental congelada, não um braço obrigatório de benchmark. O objetivo do doutorado é o framework de fine-tuning automático do Nav2. As novas comparações devem usar parâmetros de referência e parâmetros otimizados sobre **a mesma planta/interface prospectivas validadas**. Não é necessário reconstruir a cadeia histórica para comparar sua navegação com a corrigida. Se o YAML público não reproduzir o runtime histórico privado, registrar a limitação sem reatribuir os 212 resultados anteriores.

Mudar planta, interface, percepção, geometria ou modelo de ruído constitui outro fator experimental e exige novo registro de versão e nova validação relevante. Após Gate 1, o benchmark entre pares planejador–controlador é uma extensão opcional: em cada par, medir o efeito do tuning com planta e protocolo fixos. Segurança, acompanhamento de trajetória, tempo e suavidade devem ter definições operacionais congeladas antes da busca. O plano mínimo e a ordem de entrega estão em `prospective_plant_validation.md`.

Critério de conclusão desta entrega: contratos e regressões offline disponíveis, candidato reproduzível e observador de interface sem publisher de atuação, limitações/arquitetura/protocolo explícitos e nenhuma atuação conectada. Critério futuro de Gate 0: todas as precondições resolvidas, autorização específica e ensaio supervisionado com comportamento, parada e evidências aprovados pelo revisor. **Warnings eliminados, ACTIVE e IMAGE_VALIDATED não são equivalentes a Gate 0.**
