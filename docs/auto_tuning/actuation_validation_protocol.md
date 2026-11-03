# Protocolo prospectivo de atuação e demonstração supervisionada

**Estado: PREPARADO, NÃO EXECUTADO. Gate 0 e Gate 1 NOT_YET_VERIFIED.** A autorização atual cobre preparação, testes offline, probe e visualização de observação. Não cobre envio de goal, Twist, comando de junta, teste dinâmico ou conexão do conversor à planta. Este documento define as condições para uma autorização posterior; não contém comando acionável de movimento.

## Evidência e objetivo funcional

A terceira revisão leu integralmente o relatório `nav2_revisao_independente_a21403f.md`, a geometria e o grafo estruturados do ZIP `nav2_phase_a_second_review_evidence.zip` (SHA-256 `e2ffc8d0c875abc816cbccb5470ff25902e430691aba75e51cafb4c14445221d`). O runtime confirmou TF, sensores, costmaps, BTs e parâmetros obrigatórios. Preservar esses reparos; avisos cosméticos não são objetivo de engenharia.

O grafo recebido mostra **cinco publishers em `/cmd_vel`**: quatro endpoints do behavior_server e um do velocity_smoother. O Ackermann assina esse tópico; o controller publica `/cmd_vel_nav`, assinado pelo smoother. Portanto não existe hoje autoridade final exclusiva que possa garantir conversão de todas as solicitações. O limite 8,33 m/s do smoother não limita automaticamente o caminho dos behaviors. A interface física do caminhão e o runtime histórico privado não foram demonstrados como idênticos a esse plugin Gazebo.

O resultado desejado é observável: um veículo corretamente representado no mundo, um objetivo explicitamente escolhido, um caminho inspecionável, execução limitada que pode ser cancelada e interrompida, parada medida e logs suficientes para reproduzir o ensaio. Um relatório sem warnings ou com mais PASS não substitui esse resultado.

## Núcleo entregue e seu limite

`tools/auto_tuning/actuation_contract.py` contém somente matemática e uma máquina de estados **offline**. Não importa ROS, não cria publisher, não abre socket, não altera arquivos, não contém CLI de movimento e não é chamado pelo bringup. Todas as decisões retornam `runtime_authorized=false` e `physical_stop_verified=false`, inclusive no estado de modelo ARMED. Os testes usam assertions artificiais; preenchê-las com `true` não constitui aprovação real.

| API | Contrato |
|---|---|
| `Geometry.from_static_audit(audit)` | Copia geometria/limites e SHA do modelo auditado; não converte geometria estática em geometria runtime validada. |
| `Policy(...)` | Exige valores explícitos para limites, banda de parada/histerese e deadlines monotônicos. Não fornece defaults operacionais. |
| `convert(v, omega, geometry, policy)` | Recebe m/s e rad/s no ponto da bicicleta definido; devolve candidato de velocidade m/s e campo angular do plugin em rad. Nenhum objeto Twist é criado. |
| `OfflineSupervisor.arm/submit/poll/inhibit` | Modela autorização explícita, identidade fixada, autoridade única, frescor, relógios, watchdog, reversão e falha latched; não é o watchdog operacional. |

O core usa a inversa ideal `u=sign(v)*atan(L*omega/v)` para `v≠0`. O plugin depois multiplica `u` por `sign(v)` uma única vez. São testados os quatro quadrantes, zero assinado, banda de parada/histerese e ambos os joints. Curvatura acima do envelope estático é rejeitada; não há clipping de esterçamento para mascará-la. Se a política escolher redução de velocidade, v e omega são reduzidos pelo mesmo fator, preservando curvatura. Alterar somente v poderia aumentar a curvatura solicitada.

O modelo exige parada solicitada e standstill medido antes de mudar o sinal de deslocamento. Aceitar um novo pacote depois do deadline não rearma um watchdog já expirado. Dados vencidos, NaN/Inf, relógio regressivo/parado, publisher extra, source GID diferente, identidade alterada ou overspeed medido inibem a saída candidata e exigem rearmamento explícito. As falhas preservam seu motivo. Nenhum desses testes demonstra desaceleração, aderência, estabilidade, tempo de stop real ou cobertura de morte do próprio processo.

Os 32 testes incluem uma varredura de curvaturas realizáveis nos dois sentidos, reconstrução da taxa angular pelas equações do plugin, saturação conjunta, deadlines exatos, identidade alterada, bypass e stop/reversão. A confirmação de standstill é renovada com a velocidade observada atual: uma parada passada não permite ré após o veículo voltar a rolar. No checkout dedicado, este comando completo executa somente esses testes offline:

```bash
cd "$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06/gps_truck_nav"
/usr/bin/python3 -I -m unittest discover -s tests -p test_actuation_contract.py -v
```

Esses testes importam apenas stdlib e os arquivos do repositório. Os thresholds 0,05/0,10 m/s e tempos presentes nas fixtures são **valores sintéticos para exercitar ramos**, não recomendações de operação. A implementação futura precisa de limites de aceleração/desaceleração, slew de direção e margens físicas fundamentados; esse comportamento dinâmico não foi implementado no núcleo estático.

## Geometria, origem cinemática e velocidade

O cálculo estático reproduz L=6,816500073 m, W=2,598120190 m, raio traseiro 0,536 m. `max_steer=0,6458` supera o envelope dos dois joints ±0,524; a expressão conservadora resulta em ângulo virtual 0,479957 rad e raio **no eixo traseiro ideal** 13,094652 m. Isso não é uma medição de curva do Gazebo nem o raio externo da carroceria.

`base_footprint` fica 3,502 m à frente do eixo traseiro selecionado. Para deslocamento planar `r=(a,b)` entre esse eixo e a base, `v_Bx=v_R−omega*b` e `v_By=omega*a`. Com b≈0, v longitudinal coincide, mas a base tem componente lateral em curva. O MPPI Ackermann usa uma referência não holonômica; converter rad/s em rad não resolve sozinho essa diferença. Para b=0 e curva ideal, `R_base=sqrt(R_rear²+a²)`; footprint e envelope varrido ainda ampliam a região ocupada. O contrato recusa arming enquanto a referência do comando não estiver reconciliada. Não mover base, footprint, rota ou origem do evaluator silenciosamente.

Antes de decidir um raio prospectivo, medir e registrar no modelo carregado as colisões/joints selecionados, transforms, limites e origem; correlacionar essa evidência com SHA/modelo, biblioteca mapeada e versão. As APIs de leitura existentes no ZIP não incluem GetEntityState/GetModelState. Não presumir esses serviços: identificar um recurso de leitura do Gazebo transport ou preparar um plugin de ground truth separado e versionado, sem alterar a dinâmica. TF/odometria/GNSS derivados uns dos outros não constituem ground truth independente.

20 km/h = **50/9 m/s** e 25 km/h = **125/18 m/s**. São envelopes candidatos futuros; não são velocidade garantida ao longo de uma curva nem autorização para começar o primeiro ensaio a essa velocidade. O valor MPPI 25,0 m/s não foi reinterpretado como 2,5. Os perfis prospectivos mantêm o histórico e documentam divergências de caps, horizonte e consumidores. O raio 11,6 m e a referência ainda pendentes impedem tratar esses perfis como operacionalmente aprovados.

Velocidade exige também distância de parada e aceleração lateral. Para uma curva ideal, `a_y=v²/R` e uma estimativa cinemática de distância de parada inclui `v*t_latency+v²/(2*a_brake)`. Esses cálculos não calibram `a_brake`, latência, estabilidade, contato/pneus ou margens. Não escolher números para obter PASS: especificar e aprovar limites a partir de evidência antes de aplicar um envelope 20 ou 25 km/h.

## Arquitetura que precisa existir antes do primeiro movimento

Todos os produtores de navegação, inclusive behaviors, devem passar por arbitragem documentada ainda em unidades v/omega. O smoother permanece antes da conversão. O destino do plugin deve assinar um tópico exclusivo, publicado apenas pelo estágio final aprovado; nenhum tópico legado pode contornar esse estágio. GIDs e endpoints devem sustentar essa verificação em runtime, com observação renovada e detecção de perda/duplicação da autoridade.

O estágio final precisa de expiração e parada independentes da continuidade do controller/smoother/conversor e do avanço do clock simulado. No upstream inspecionado, o callback Ackermann mantém o último alvo e não implementa timeout. Uma publicação zero emitida pelo conversor antes de morrer não resolve o caso em que ele morre antes de publicar. A futura solução no destino deve ser prospectiva, isolada, testada por injeção de falhas e registrada por versão; não editar `/opt/ros/iron` nem afirmar que este core já oferece essa proteção.

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

Manter duas comparações separadas: (1) histórico funcional como referência documental congelada, com suas limitações de reprodutibilidade; (2) baseline prospectiva e variantes de parâmetros sobre **a mesma planta/interface validadas**. Não executar uma interface conhecida incompatível como controle A apenas para obter um contraste favorável. Se a baseline pública não for equivalente ao runtime histórico privado, registrar a diferença em vez de atribuir a ela os 212 resultados anteriores. Mudar planta, interface, percepção, geometria ou modelo de ruído constitui outro fator experimental e exige protocolo separado.

Critério de conclusão desta entrega: core e regressões offline disponíveis, limitações/arquitetura/protocolo explícitos e nenhuma atuação conectada. Critério futuro de Gate 0: todas as precondições resolvidas, autorização específica e ensaio supervisionado com comportamento, parada e evidências aprovados pelo revisor. **Warnings eliminados, ACTIVE e IMAGE_VALIDATED não são equivalentes a Gate 0.**
