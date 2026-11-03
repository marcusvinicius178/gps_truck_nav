# Engineering handoff — Fase A, 03/11/2026

## Resultado e limite

**HOST PREFLIGHT APROVADO pelo revisor para ipt-sim-01. Gate 0 e Gate 1: NOT_YET_VERIFIED.** O ZIP do commit `a21403f` confirma IMAGE_VALIDATED, TF/frames corrigidos, odometria/juntas preservadas, costmaps/BTs/lifecycle e parâmetros obrigatórios. São 69 PASS, zero FAIL, 3 UNKNOWN, 10 BLOCKED_REVIEW e `mission_ready=false`. A captura PARTIAL resulta de instrumentação, não demonstra falha de navegação. O ZIP completo e o terceiro relatório independente foram lidos antes das alterações relevantes. Neste Work não houve build Docker nem lançamento ROS/Gazebo; testes compilam somente o núcleo C++ sem ROS. Não houve missão, gravação MCAP nova ou otimização. A decisão de 14/10 permanece CONDITIONAL GO.

O proprietário já executou `host_preflight.py` e forneceu o JSON. A leitura integral confirmou Ubuntu 22.04.5, 32 CPUs lógicas, 60,3 GiB RAM, Docker 29.8.2/Compose 5.5.1, daemon local e Git/LFS existentes. Humble deve permanecer intacto; não instalar Gazebo no host. Detalhes e limites estão em `host_preflight_ipt_sim_01.md`.

O próximo passo está na seção atual de escolha da planta em `local_setup_runbook.md`: atualizar o checkout dedicado, reconstruir a imagem e, somente após IMAGE_VALIDATED, repetir os probes indicados e abrir a observação visual. A investigação atual está em `third_independent_review.md`, `speed_limits_provenance.md`, `actuation_validation_protocol.md` e `visual_observation_runbook.md`. Os arquivos completos estão no branch da PR #2; não há edição manual por trechos. Retornar o novo ZIP ao revisor independente. Falha de build interrompe a sequência e preserva os logs.

Os perfis 20/25 km/h têm YAMLs completos e readback próprio, sem alteração dos valores históricos. A escolha incremental é manter temporariamente Gazebo Classic 11 e preparar uma pequena interface ROS 2, sem modificar o plugin upstream. O core estático e o observador ROS sem publisher de atuação não estão ligados ao veículo. O candidato separa o tópico de atuação, reconcilia a origem cinemática no eixo traseiro, o raio estático e os envelopes prospectivos; comportamento dinâmico e interrupção ainda dependem de ensaios autorizados. O visor mostra Gazebo/RViz via imagens em localhost, sem canal de entrada para o container. Preparar demonstração observável é a prioridade; nenhuma limpeza cosmética de warnings ou contagem de PASS substitui comportamento funcional, parada medida ou validade científica.

## Escopo científico atualizado

O produto principal do doutorado é um framework de fine-tuning automático de parâmetros Nav2 sobre uma planta Mercedes Arocs validada e fisicamente coerente. **Não é requerido benchmark planta histórica versus corrigida.** Arquivos, imagens e experimentos históricos ficam preservados para rastreabilidade; a referência operacional dos novos experimentos será a planta prospectiva após sua validação.

A comparação científica principal é parâmetros de referência versus parâmetros otimizados com a planta/interface fixas. Se houver tempo, estender a comparação a outros pares planejador–controlador, mantendo a mesma planta dentro de cada comparação e métricas de segurança, acompanhamento, tempo e suavidade previamente definidas. Um reparo da planta não pode ser creditado ao tuning. O benchmark opcional não deve adiar a demonstração funcional inicial.

Ordem de entrega: fechar o contrato estático e validar atuação → observar Gazebo/RViz → autorizar e executar um ensaio supervisionado limitado → Gate 0 → Gate 1/reset/métricas → framework → benchmark opcional. `prospective_plant_validation.md` transforma essa ordem em critérios verificáveis. As evidências já aprovadas de TF, sensores, costmaps e BTs são preservadas; apenas regressões ou falhas com impacto demonstrado reabrem esses itens.

## Entregáveis

| Arquivo relativo à raiz do repositório | Conteúdo |
|---|---|
| `docs/auto_tuning/environment_preflight_report.md` | Bases, dependências, Docker/LFS, requisitos e bloqueios versionados. |
| `docs/auto_tuning/host_preflight_ipt_sim_01.md` | Diagnóstico fornecido pelo proprietário, aprovação do revisor e limites dessa evidência. |
| `docs/auto_tuning/local_setup_runbook.md` | Comandos completos, diretórios/pré-requisitos, build de descoberta e bringup diagnóstico com cleanup por ID. |
| `docs/auto_tuning/gate0_gate1_plan.md` | Estados separados, prontidão, gravação, associação de referência, aceitação, teardown/reset e pendências. |
| `docs/auto_tuning/virtual_lanes_technical_roadmap.md` | Auditoria e plano independente VL0–VL5, com fontes Iron e matriz de testes. |
| `docs/auto_tuning/metrics_adapter_contract.md` | Interfaces reais do evaluator, referências, schema e limites do adapter. |
| `tools/auto_tuning/host_preflight.py` | Diagnóstico local, timeouts, recursos/cgroup, Git/LFS e daemon Docker opcional restrito a socket Unix. |
| `tools/auto_tuning/local_phase_a.py` | Build/probe locais separados, stdlib, daemon/builder fixos, fonte/imagem congeladas, cleanup por ID e evidências. |
| `tools/auto_tuning/validate_image.sh` | Smoke checks sem lançar simulação: versão, imports, MCAP registrado, bibliotecas, XML e parsing do launch. |
| `tools/auto_tuning/prepare_candidates.py` | YAMLs completos A/B, alteração única 2,5→2,6 e manifest de proveniência. |
| `tools/auto_tuning/evaluate_trial.py` | Um bag prospectivo, evaluator congelado, validação de hashes/IDs/flags e falhas de medição sem scores fabricados. |
| `tools/auto_tuning/collect_runtime_evidence.sh` | Probes ROS limitados no container, somente observações, sem aprovação automática. |
| `docker/auto_tuning/Dockerfile` e `Dockerfile.dockerignore` | Imagem prospectiva com base fornecida por digest, guard LFS, build truck_bringup e MCAP. |
| `config/auto_tuning/dual_ekf_navsat_gate0.yaml` | YAML completo de localização, quatro mudanças isoladas e explícitas. |
| `config/auto_tuning/models/arocs_truck_tf_sensors/`, `worlds/ctvi_tf_sensors.world`, `tf_sensor_model_variant.json` | Variante completa com apenas três alterações SDF, mundo selecionado explicitamente e hashes/pais/lever arms preservados. |
| `tools/auto_tuning/parameter_probe.py`, `plugin_parameter_contract.py` | Readbacks tipados e limitados, evidência parcial preservada, identidade e consumidor separados de execução algorítmica. |
| `tools/auto_tuning/tf_timing_contract.py`, `localization_alignment.py`, `runtime_binary_identity.py`, `audit_ackermann_geometry.py` | Clock da mesma janela, comparações exatas de stamp, identidade de bibliotecas mapeadas e contrato estático sem atuação. |
| `docs/auto_tuning/second_independent_review.md` e três contratos técnicos associados | Causas, limites, política de covariância, caminho de atuação e critérios verificáveis de fechamento. |
| `tests/test_host_preflight.py`, `test_local_phase_a.py`, `test_prepare_candidates.py`, `test_evaluate_trial.py`, `test_gate0_localization_contract.py` | Regressões sem dependência de runtime ROS; Docker simulado por fixtures nos testes do helper. |
| `tools/auto_tuning/ackermann_adapter.py` | Observador ROS sem publisher de atuação: entrada, clock/odom, proveniência e candidatos condicionais em JSON. Sem opção live. |
| `config/auto_tuning/plant_candidate/` | Bundle completo prospectivo: SDF/world/URDF, frame `rear_axle_nav`, perfis 20/25 km/h, política shadow e manifest reproduzível; sem alterar a geometria histórica. |
| `docs/auto_tuning/prospective_plant_validation.md` | Escopo científico atualizado e sequência mínima até demonstração, Gates e tuning; benchmark histórico/corrigido dispensado. |
| `docs/auto_tuning/evidence/` | Logs reais de testes/audit/LFS/sintaxe e resumo de proveniência. |

Arquivo existente modificado: `src/gps_truck_nav/truck_bringup/launch/gps_waypoint_follower.launch.py`. A declaração `publish_legacy_cloud_tf` fica junto dos argumentos `use_gpu/use_xcb`; sua condição fica no Node `static_transform_cloud`. Default true preserva chamadas antigas; novas chamadas de diagnóstico usam false. Não é necessário inserir trechos manualmente: os arquivos do branch estão completos.

## Execução comprovada na preparação inicial

Bases: integração `19f38e535bebc8a03a6aeef75c29160e0524668b`; análise `36ba76823cc7a124540412cf3ce09c73e6cd1d05`. Ambas iguais às examinadas em 14/10. O relatório anterior foi lido integralmente.

- **56 testes PASS:** 7 testes existentes, 5 contratos localização/TF, 12 diagnóstico do host, 10 preparação A/B e 22 adapter de métricas. Fixtures são artificiais; não são runs científicos.
- `git lfs pull`: sete meshes recuperadas, 277.750.112 bytes; `git lfs fsck`: OK.
- Audit estrutural: **PASS**, 15 YAMLs, 49 arquivos Python/launch, 35 XMLs, três pacotes e oito entrypoints; todas as referências de mesh verificadas após hidratação.
- Sintaxe dos oito blocos Bash do runbook e do coletor: PASS. Guard Python do Dockerfile compilado e exercitado nas meshes locais hidratadas; isso não é um build Docker.
- Geração A/B executada sobre YAML canônico; manifests registram `effective_value_verified=false` e gates não verificados.
- `git diff --check`: PASS. Checkout de métricas permanece sem alterações; YAMLs canônicos de navegação/localização, SDF/URDF e materiais históricos preservados.

Diagnóstico real do Work: Ubuntu 24.04.3, x86_64, quota visível 8 cores/8 GiB; Docker/ROS/Gazebo ausentes. Não existe acesso estabelecido ao computador de Marcus.

## Continuação após aprovação do host

O novo fluxo não usa Compose, volumes de build anteriores, rede do host ou GPU. Runtime usa 8 CPUs como quota, 16 GiB como limite e Xvfb/Mesa dentro da imagem; esses limites não reservam capacidade. Build limita jobs CMake/Make, mas não impõe teto global ao BuildKit. O helper mede espaço de DockerRootDir separadamente do diretório de evidências, registra digest da base, image ID, pacotes e hashes dos inputs, e recusa fonte alterada ou inputs ignorados que poderiam escapar do manifesto.

O build valida pré-requisitos da imagem. O probe coleta observações e para somente seu container identificado por ID e labels; confirma `Running=false`/`Pid=0`. Isso não comprova reset de um experimento. A ausência de runtime neste Work permanece: resultados de testes offline e sintaxe da continuação constam em `evidence/host_continuation_summary.json` e seus logs. Não foram fabricados logs Docker/ROS de ipt-sim-01.

Verificação da continuação: **71 testes PASS**, incluindo 15 testes do novo helper com Docker simulado; auditoria estrutural PASS (51 Python, 15 YAML, 35 XML); cinco blocos Bash do runbook e Python embutido do validador compilados/verificados. Revisão independente conferiu isolamento, manifesto de fonte, cleanup e falhas de coleta. Os quatro hashes canônicos protegidos permanecem iguais; checkout de análise limpo. Essas verificações não executam o Docker nem substituem os resultados locais.

## Bloqueios que impedem missão agora

1. **Interface de comando:** core puro e supervisor offline testam conversão, ré/zero, saturação, autoridade e falhas; observador ROS não publica atuação. O candidato prospectivo reconcilia frame no eixo traseiro, raio 13,10 m derivado dos limites estáticos e tópico privado sem publishers. A semântica do PID da roda traseira direita exige também correção do alvo linear em curva. Watchdog final, autoridade exclusiva operacional, medições runtime e atuação autorizada posterior continuam necessários. Nenhum conversor foi ligado ao veículo.
2. **Localização absoluta:** TF, headers e consistência interna GNSS/TF foram confirmados em a21403f; não reabrir sem regressão. Covariâncias e precisão contra referência independente permanecem pendentes; risco temporal no source navsat ainda exige correlação ao binário. Nenhum datum, geometria ou ruído foi substituído.
3. **Percepção/parâmetros:** costmaps, BTs e readbacks obrigatórios Smac/MPPI estão confirmados. O produtor de `/external_area_pointcloud` não foi identificado; não habilitar Virtual Lanes para mascará-lo. Opcionais comprovadamente ausentes são distintos de RPCs com falha; nomes sem consumidor continuam fora da hipótese de tuning. Perfis 20/25 km/h precisam do novo readback e continuam sem autorização de atuação.
4. **Missão/referência/velocidade:** rota ativa pública marcada 5_6 não pode ser assumida S1/S2/S3. Os limites atuais não definem automaticamente 20 km/h. O equivalente de steering do evaluator deve ser conciliado com o modelo.
5. **Ambiente/medição:** imagem anterior identificada e validada no host; a imagem corrigida, incluindo compilação do adaptador ROS do observador, ainda depende de build local. `dpkg-query`, digest e logs continuam congelados por tentativa.
6. **Automação:** a coleta agora produz critérios objetivos e `mission_ready=false`; faltam supervisor de missão, action JSON, recorder e reset entre trials de referência e parâmetros otimizados sobre a mesma planta. O diagnóstico não os substitui.

## Revisão e próximo passo

A revisão independente deve avaliar o escopo de preparação, a hipótese de reparo de localização e os bloqueios acima. A PR é **draft**, para revisão, sem merge. Não representa entrega do micro-PoC.

Sequência seguinte: build/validação da imagem corrigida → probe diagnóstico → revisão independente das novas evidências → resolver atuação/sensores e obter autorização explícita antes de qualquer missão. Gate 0, supervisor, reset comparável e busca permanecem etapas futuras. Não há Optuna instalado/configurado nem campanha de simulação nesta entrega.

Os 212 runs históricos, suas métricas e a tese permanecem fora deste trabalho. Virtual Lanes segue independente e ainda não foi validada como restrição de navegação.
