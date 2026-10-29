# Engineering handoff — Fase A, 29/10/2026

## Resultado e limite

**HOST PREFLIGHT APROVADO pelo revisor para ipt-sim-01. Gate 0 e Gate 1: NOT YET VERIFIED.** Auditoria estática e preparação entregues. Não houve build de imagem, build C++, lançamento ROS/Gazebo, missão, gravação MCAP nova ou otimização neste Work. A decisão de 14/10 permanece CONDITIONAL GO; estes testes não a convertem em aprovação de runtime.

O proprietário já executou `host_preflight.py` e forneceu o JSON. A leitura integral confirmou Ubuntu 22.04.5, 32 CPUs lógicas, 60,3 GiB RAM, Docker 29.8.2/Compose 5.5.1, daemon local e Git/LFS existentes. Humble deve permanecer intacto; não instalar Gazebo no host. Detalhes e limites estão em `host_preflight_ipt_sim_01.md`.

O próximo passo é a seção 2 de `local_setup_runbook.md`: checkout novo e `local_phase_a.py build`. Somente após `IMAGE_VALIDATED`, executar o estágio `probe` da seção 3, sem missão. Os arquivos completos estão no branch da PR #2; não há edição manual por trechos. Retornar o ZIP de evidências da seção 4. Falha de build interrompe a sequência e preserva os logs.

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
| `tests/test_host_preflight.py`, `test_local_phase_a.py`, `test_prepare_candidates.py`, `test_evaluate_trial.py`, `test_gate0_localization_contract.py` | Regressões sem dependência de runtime ROS; Docker simulado por fixtures nos testes do helper. |
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

1. **Interface de comando:** Gazebo Ackermann 3.7.0 interpreta angular.z como esterçamento; MPPI usa yaw rate. Confirmar binário real e conversão antes de dirigir. Não foi implementado conversor com wheelbase presumida nem alegado defeito do runtime histórico privado.
2. **TF/localização:** perfil novo resolve estaticamente duplicação map→odom, input GPS e TF cloud; convergência/autoridades/sensores ainda não foram observados. GPS/IMU têm risco de frame/lever arm, e GPS tem remaps duplicadas.
3. **BT/costmaps:** verificar Spin no BT padrão e a observação `/external_area_pointcloud` ausente. Não iniciar Virtual Lanes para mascarar dependência do core.
4. **Missão/referência/velocidade:** rota ativa pública marcada 5_6 não pode ser assumida S1/S2/S3. Os limites atuais não definem automaticamente 20 km/h. O equivalente de steering do evaluator deve ser conciliado com o modelo.
5. **Ambiente/medição:** tag/APT, build/load, renderer headless e MCAP Iron ainda dependem de execução. `dpkg-query` e imagem final devem ser congelados.
6. **Automação:** faltam supervisor de missão, readiness automático, action JSON, recorder e reset entre A/B. O coletor diagnóstico não os substitui.

## Revisão e próximo passo

A revisão independente deve avaliar o escopo de preparação, a hipótese de reparo de localização e os bloqueios acima. A PR é **draft**, para revisão, sem merge. Não representa entrega do micro-PoC.

Sequência seguinte, partindo do host aprovado: build/validação da imagem → bringup diagnóstico → resolver atuação/sensores/BT com versão efetiva → definir missão/referência → Gate 0 com dados novos → implementar supervisor → executar A/B e reset → revisão independente obrigatória antes de qualquer busca. Falha de reset interrompe a sequência. Não há Optuna instalado/configurado nem campanha de simulação nesta entrega.

Os 212 runs históricos, suas métricas e a tese permanecem fora deste trabalho. Virtual Lanes segue independente e ainda não foi validada como restrição de navegação.
