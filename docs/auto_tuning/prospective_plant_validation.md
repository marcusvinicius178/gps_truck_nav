# Planta prospectiva e caminho mínimo até a navegação supervisionada

**Preparação incremental; sem autorização de movimento. Gate 0 e Gate 1: NOT_YET_VERIFIED.** Este protocolo complementa o runbook de execução local e `actuation_validation_protocol.md`. Não contém comandos para enviar goal, Twist, comando de junta, missão ou otimização. A execução dinâmica depende de autorização posterior que identifique a tentativa e seus limites.

## Pergunta científica e unidade de comparação

O objetivo principal é desenvolver e avaliar um framework de fine-tuning automático de parâmetros do Nav2. A planta Mercedes Arocs deve ser fisicamente coerente, reproduzível e validada para o envelope utilizado. **Não há obrigação de benchmark entre a planta histórica e a prospectiva.** Preservam-se o histórico, os 212 experimentos e suas limitações de reprodutibilidade; eles não serão reexecutados apenas para justificar a correção da interface.

Cada comparação nova deverá congelar a planta/interface e contrastar parâmetros de referência com parâmetros otimizados. Registrar separadamente planta, configuração Nav2, cenário, velocidade experimental e versão do avaliador. Se algum desses fatores mudar, não atribuir o resultado exclusivamente ao tuning. Os envelopes 20 e 25 km/h constituem condições experimentais distintas, não duas observações intercambiáveis da mesma condição.

O benchmark entre pares planejador–controlador é uma extensão condicionada ao tempo. Para cada par, estabelecer uma configuração de referência válida e um espaço de busca cujo consumo efetivo pelos plugins tenha sido demonstrado. Manter fixos planta, interface, percepção, ruído, cenário, reset e métricas dentro de cada comparação. Falhas, timeout e saturações são resultados; não excluí-los silenciosamente nem fabricar scores.

## O que precisa ser fechado agora

A decisão é **manter temporariamente o Gazebo Classic 11 e preparar um adaptador ROS 2 pequeno**, sem modificar o plugin upstream. É a intervenção de menor alcance sobre o ambiente já observado: conserva os sensores, TF e cadeia de odometria conhecidos. O incremento disponível recebe e analisa comandos, sem publicar atuação. Um candidato com tópico privado sem publishers permite observar o modelo sem conectar o conversor ainda não validado.

O `AckermannSteering` moderno interpreta v/omega, mas migrar não elimina a validação. Exige verificar conversão de sensores/plugins, integração ROS/Docker, física, controle das juntas e odometria; a implementação moderna inspecionada também não fornece expiração de comandos por si só. Uma mudança de simulador agora amplia o conjunto de fatores alterados sem evidência de maior fidelidade do Arocs. Reavaliar a migração se a caracterização demonstrar um defeito de dinâmica do Classic que o adaptador não possa corrigir, ou se a manutenção da imagem congelada impedir sua reprodução. Não abrir uma migração em paralelo como condição de Gate 0.

A conversão correta de unidades resolve apenas a interface estática: referência cinemática, limites, comportamento dinâmico e interrupção continuam exigindo evidência. Para o Classic, corrigir também o alvo linear do PID da roda traseira direita (`v_plugin=v_R−omega*y_right`). O candidato explicita `rear_axle_nav`, raio mínimo ideal 13,10 m, limite virtual derivado e perfis 20/25 km/h inicialmente à frente. O arredondamento de 13,094652 para 13,10 m assegura compatibilidade matemática com os joints; não é margem dinâmica de segurança. Modelo histórico, datum e geometria física ficam preservados.

O bundle completo fica em `config/auto_tuning/plant_candidate/`: SDF/world/URDF, perfis `navigation/candidate20kmh.yaml` e `candidate25kmh.yaml`, políticas de observação e manifest. O frame adicional e o footprint traduzido representam a mesma carroceria; objetivos e métricas do candidato referem-se ao eixo traseiro, sem reutilizar o offset frontal do checker histórico. Os artefatos anteriores permanecem disponíveis. `SHADOW_COMPLETE` e `NOT_EXERCISED` no observador não liberam movimento: o primeiro qualifica a coleta; o segundo é esperado quando nenhum comando foi emitido.


| Contrato | Fechamento estático verificável | Evidência dinâmica posterior |
|---|---|---|
| Comando | Documentar fonte/versionamento; v em m/s, yaw rate em rad/s e comando angular do plugin em rad; inversa testada nos quatro quadrantes, zero e entradas inválidas. | Reconstituir v/omega pedidos e medidos, sinal em ré, ângulos das duas rodas e curvatura realizada. |
| Geometria e origem | Identificar colisões/joints usados pelo plugin, distância entre eixos, bitola, raio de rodas, limites e ponto de referência. Modelos históricos ficam intactos; qualquer variante tem deltas e hashes explícitos. | Medir pose e juntas do modelo carregado, conferir referência e registrar o erro entre curva ideal e realizada. |
| Raio e velocidade | Não pedir raio incompatível com o envelope estático; MPPI, planner, smoother e interface final têm contratos coerentes, comprovados por readback. | Confirmar que velocidade, curvatura, acelerações, saturações e parada permanecem dentro do envelope previamente aprovado. |
| Autoridade e interrupção | Topologia prevista tem destino exclusivo, sem bypass de behaviors; timeouts são monotônicos e independem de `/clock`. A morte do estágio final tem contenção externa definida. | Injetar perda dos produtores/conversor, clock parado e concorrência em ensaio delimitado; provar término e medir a parada nos casos em que ela é exigida. |
| Percepção/localização | Preservar TF/costmaps/BTs já demonstrados; registrar covariâncias e fontes realmente disponíveis, sem inventar precisão nem desligar proteção para obter PASS. | Antes da navegação, confirmar obstáculo conhecido e posição/rota contra referência independente no cenário do ensaio. |

Os valores 11,6 m e 25,0 m/s históricos não são corrigidos como erros tipográficos. O raio prospectivo deriva da geometria e da referência escolhidas; 20/25 km/h são tetos experimentais solicitados, não capacidades validadas do caminhão. Limites herdados de aceleração, frenagem e ruído continuam identificados como hipóteses até haver medição. Não alterar datum, massas, colisões ou ruído silenciosamente.

## Sequência de entrega com condições de saída

O próximo ciclo local deve reunir build, probe e observação visual em uma única tentativa identificável, preservando cada evidência. Se aparecer uma falha de instrumentação sem efeito demonstrado na decisão, registrar sua limitação e corrigir somente o trecho necessário. Não repetir a validação inteira de TF, costmaps ou BTs já aprovadas por causa de um aviso cosmético.

| Etapa | Entrada | Resultado que encerra a etapa |
|---|---|---|
| 1 — preparação desta revisão | Fontes oficiais, modelo versionado e testes offline. | Decisão de interface fundamentada, candidato reproduzível, contratos e diferenças registrados; nenhuma atuação conectada. |
| 2 — imagem e observação | Build no `ipt-sim-01`; avanço somente após `IMAGE_VALIDATED`. | Probe da mesma imagem/fonte, valores efetivos, TF/clock e Gazebo/RViz visíveis; evidências entregues ao revisor. `OBSERVATIONS_COLLECTED` não significa planta validada. |
| 3 — autorização de comissionamento | Contratos estáticos, topologia exclusiva e contenção aprovados; registro de ensaio preenchido. | Autorização explícita para testes curtos de atuação, com envelope e condições de interrupção próprios. A autorização atual não cobre esta etapa. |
| 4 — calibração e interrupções | Área livre, ground truth e operador presentes; ensaio autorizado. | Reta, curvas pequenas e ré caracterizadas; parada e falhas do caminho de comando medidas; comportamento consistente com a geometria. Sem missão de navegação ainda. |
| 5 — primeiro objetivo supervisionado | Atuação e percepção/localização adequadas ao ensaio; autorização específica. | Seleção do objetivo separada da execução, caminho inspecionado, acompanhamento do caminhão e cancelamento/interrupção disponíveis; limite de duração/distância cumprido. Evidência ao revisor para Gate 0. |
| 6 — repetição automatizável | Gate 0 aprovado. | Dois trials com reset comprovado, parâmetros efetivos, bag/métricas íntegros e falhas identificadas; revisão de Gate 1. |
| 7 — fine-tuning | Gate 1 aprovado e planta congelada. | Referência versus otimizado, separação treino/validação, registro de seeds e orçamento; benchmark adicional apenas se couber no escopo. |

As etapas são condições de avanço, não um novo programa de benchmarking da planta. O primeiro objetivo é pequeno e supervisionado; aumentar até 20 km/h exige antes demonstrar espaço de parada e estabilidade no percurso. O envelope 25 km/h é uma extensão posterior na mesma planta, não requisito para o primeiro Gate 0.

## Registro mínimo antes de qualquer ensaio dinâmico

Um registro versionado deve conter commit/tree, image ID/digest, hashes do modelo/plugins/configurações, referência cinemática, cenário e área autorizada, operador, autorização, hipótese, direção permitida, teto de velocidade, curvaturas e limites de tempo/distância. Identificar também observadores, taxa/resolução efetivas, timeouts, critério de standstill, critério de erro, distância de abortagem, mecanismos de cancelamento/interrupção e reset. Valores ausentes permanecem pendentes; não usar thresholds das fixtures de unit test como política de operação.

A definição de tolerâncias precede a avaliação da tentativa. Fundamentá-las na resolução de timestamp/pose/juntas e taxa observada dos sensores, precisão da referência independente, distância disponível e incerteza do modelo. Por exemplo, um erro de curvatura só é interpretável com velocidade diferente de zero, janela de observação suficiente e distinção entre eixo traseiro, base e envelope da carroceria. Registrar o cálculo e a justificativa da tolerância; não ajustá-la depois de ver um resultado para obter PASS. Se a medição revelar que a tolerância era inadequada, versionar o protocolo e repetir o caso identificado.

Na caracterização inicial, guardar sinais pedidos e efetivos, e não apenas o retorno da API: v/omega antes e depois do smoother, conversão candidata/aplicada, ângulos de ambas as rodas, velocidade/yaw rate/pose de referência, saturações, clock simulado e recepção monotônica. Morte do adaptador e publisher concorrente não devem reaproveitar o último alvo indefinidamente. Encerrar o container próprio é contenção da simulação, **não prova de frenagem física**; registrar esse caso como abortado e preservar logs.

## Demonstração visual e funcional

A observação atual usa os aplicativos reais Gazebo Classic/RViz numa sessão privada e apresenta seus pixels sem canal de entrada. Confirmar Arocs, meshes, sensores, TF, footprint, costmaps e avanço do relógio. Um display de trajetória vazio antes de planejar é esperado; não enviar goal para preencher a tela. As instruções completas estão em `visual_observation_runbook.md` e no runbook principal. A imagem de aproximadamente 1 Hz serve à observação, não à medição sincronizada de parada ou latência.

Na etapa autorizada de navegação, a interface deverá distinguir três ações: selecionar objetivo; planejar e inspecionar; autorizar execução. O operador deverá poder ver goal/frame, caminho, ocupação e footprint, velocidade pedida/aplicada/medida, estado da ação e inibição. O modo atual de observação não oferece esses controles e não deve ser apresentado como pronto para conduzir um teste. Cancelar a ação e confirmar standstill são eventos distintos; a interrupção independente permanece disponível durante todo o ensaio.

## Critérios de aceite e encerramento do diagnóstico

Os gates permanecem `NOT_YET_VERIFIED` até a evidência da etapa correspondente ser aprovada. Três classes de conclusão evitam prolongar a fase de diagnóstico:

- **Resolvido no escopo estático:** contrato reproduzível e testes sustentam a transformação/configuração, com limites de interpretação explícitos. Não exige um novo benchmark histórico.
- **A confirmar no próximo runtime:** arquivo carregado, publisher/assinante, readback, identidade de biblioteca, imagem e visualização dependem do `ipt-sim-01`. Uma coleta objetiva deve decidir esse ponto ou identificar precisamente a falta de evidência.
- **Condicionado ao ensaio autorizado:** curvatura realizada, estabilidade, frenagem e segurança de navegação não podem ser aprovadas por ausência de warnings. Necessitam do ensaio limitado, não de mais provas de existência de YAML.

Após fechar os contratos que realmente afetam a atuação e a interpretação dos resultados, solicitar a autorização específica do comissionamento. Não transformar o benchmark opcional, a reconstrução da planta histórica ou a eliminação de todos os warnings em pré-requisitos adicionais. Preservar Humble, Docker, CARLA/Autoware, Overleaf e os experimentos anteriores. A PR #2 permanece DRAFT, sem merge.

## Fontes e documentos associados

- [Plugin Classic `gazebo_ros_ackermann_drive`, versão 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp): semântica dos comandos, controle das rodas e ausência de expiração de comandos na implementação inspecionada.
- [API oficial `AckermannSteering` do Gazebo Sim 8](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html): interface e parâmetros do sistema moderno; a API documentada não substitui a inspeção da versão de source escolhida nem a caracterização da planta migrada.
- `ackermann_interface_contract.md`: auditoria do modelo e distinção entre fonte oficial, biblioteca instalada/mapeada e execução.
- `actuation_validation_protocol.md`: matemática, condições de autorização, medições e interrupção.
- `speed_limits_provenance.md`: histórico dos valores e comparação científica com planta fixa.
