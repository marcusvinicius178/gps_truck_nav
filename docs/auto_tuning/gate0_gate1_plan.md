# Gates 0 e 1 — protocolo de execução a fechar

**Estado atual: NOT YET VERIFIED.** Existe preparação de candidatos, diagnóstico e adaptador; o supervisor de missão/gravação/reset ainda não está implementado. O roteiro local existente executa somente bringup diagnóstico. As decisões abaixo precisam ser fixadas antes dos novos runs, com evidências próprias. Não executar optimizer.

## Gate 0: uma missão curta, quatro resultados separados

| Etapa | Evidência exigida | Bloqueio |
|---|---|---|
| Ambiente | SHAs, base digest, image ID, imagem preservada, pacote/ABI, hashes de world/model/YAML/BT, LFS hidratado. | Build/pull/plugin failure = infraestrutura. |
| Prontidão | Clock avançando, pose inicial, TF única map→odom→base_footprint e sensor frames, timestamps válidos, GPS/navsat/fromLL, lifecycle active, costmaps current. | Timeout/TF/sensor/costmap failure impede envio de goals. |
| Interface Ackermann | Semântica real de cmd_vel/plugin, conversão yaw rate→esterçamento validada com geometria, ré/zero/saturação/watchdog e comandos observados. | Upstream Gazebo 3.7.0 consome esterçamento; pipeline pública não apresenta conversor. Não enviar missão com essa incompatibilidade. |
| Missão | Rota convertida preservada, origem/datum/offset, limites temporais, nominal speed e limites físicos; action accepted/result/feedback em JSON. | Missão pública não possui associação comprovada aos CSVs S1–S3. |
| Registro | Um MCAP finalizado, sem split, QoS correto e mensagens efetivamente presentes; registrar início/fim e falhas do recorder. | Processo vivo não prova gravação. |
| Geometria | PR contra referência comum associada à missão, com hash e revisão antes da execução; success_geo separado da action. | Não calcular progresso de uma rota curta contra referência inteira arbitrária. |
| Métricas | Adapter extrai uma linha, valida falhas e flags; artefatos/hash preservados. | Extração bem-sucedida não implica tracking elegível nem missão concluída. |
| Encerramento | Recorder finalizado, container/processos próprios parados, logs e estado retidos. | Falha de teardown bloqueia próxima tentativa. |

Reportar independentemente: `simulation_launched`, `action_status`, `geometric_success`, `metrics_status`. Para anunciar reprodução bem-sucedida, exigir lançamento válido, action SUCCEEDED, critério geométrico satisfeito e extração válida; se uma condição falhar, manter as outras observações sem colapsá-las em “deu certo”. Falha real de navegação pode ter métricas válidas e continua distinta de infraestrutura.

## Prontidão a implementar no supervisor

1. Tempo limite global de startup em relógio monotônico do host; `/clock` não pode controlar timeout de startup. Valor inicial proposto 180 s, congelado após medir bringup, sem aumentos automáticos para ocultar falhas.
2. Health probes contínuos para launch, gzserver, componentes Nav2, recorder e serviços; processo launch vivo não prova que filhos continuam vivos.
3. Active lifecycle para planner/controller/bt_navigator/behavior/smoother/waypoint/velocity smoother conforme grafo efetivo. Confirmar plugins carregados nos logs.
4. Amostras repetidas de clock, odometria, GPS e TF, com idade/avanço medidos no mesmo domínio temporal; um `topic echo --once` é somente diagnóstico. GPS do modelo é 1 Hz: fixar tolerância compatível, sem exigir 30 Hz de todo sensor.
5. TF deve ter única autoridade por aresta; verificar autoridade e conteúdo, incluindo caches de transient_local. Validar que fromLL converte pontos conhecidos para coordenadas coerentes com world e pose.
6. Dump completo de parâmetros, comparação dos knobs esperados, físicos e BT. Confirmar runtime effect do knob na versão exata; ignorar parâmetros não consumidos é falha de configuração.
7. Confirmar nenhum goal anterior, velocidade próxima de zero, modelo na pose inicial e costmaps reinicializados. Fixar tolerâncias a partir da arquitetura e medição, não escolher após observar resultado favorável.

O coletor `collect_runtime_evidence.sh` ajuda a diagnosticar essas condições, mas não implementa esse gate automático.

## Protocolo e entradas a congelar

- Um cenário curto novo, rota enviada e referência de progresso associadas, antes de medir o candidato. Se usar uma condição S1/S2/S3, comprovar correspondência; não inferir por nome informal 5_6.
- A associação rota/referência deve ser revisada antes de executar a missão. Após finalizar o bag, o manifest `schema_version=2` incorpora essa associação congelada, `dataset_kind=prospective_auto_tuning`, `scenario_id`, `speed_kmh`, SHA256 da referência/rota/metadata/MCAP, `trial_id`, `attempt_id` e `association_evidence` com a revisão. O adapter exige correspondência com os IDs da CLI e hashes dos inputs. Isso vincula arquivos, mas não autentica sozinho a origem prospectiva; o supervisor deve garantir criação nova. O adapter aceita IDs 1–3; para cenário completamente novo, definir explicitamente as regras geométricas antes de estender o adapter.
- Nominal speed é metadado do protocolo e deve corresponder ao limite efetivo, não apenas ao argumento do evaluator. Preservar/validar raios Smac/MPPI de 11,6 m, footprint, datum e limites de atuação.
- `wheelbase_m` no adapter é equivalente de steering, não um parâmetro otimizado; reconciliar 6,804 m do evaluator com 5,950 m de outro contexto antes de comparar steering-rate.
- Definir duração limite de missão e critérios de ausência de progresso antes dos runs. Proposta inicial de missão curta: teto 300 s de wall time, a confirmar pela extensão/velocidade. Startup timeout e mission timeout são classes diferentes.
- Seed Gazebo/MPPI: registrar por componente. O launch atual não expõe seed e MPPI não tem seed ROS comprovada; usar `null/uncontrolled`, não alegar determinismo.
- Collision truth indisponível: registrar `not_instrumented`. Ausência de evento sem detector não é ausência de colisão; não tirar conclusão de segurança ou liberar tarefa com obstáculos com esse protocolo.

## Registro e extração

Tópicos a conferir no grafo e gravar: `/clock`, `/odometry/global`, `/odometry/local`, `/odometry/gps`, `/odom`, `/plan`, `/received_global_plan`, `/transformed_global_plan`, `/plan_smoothed`, `/cmd_vel_nav`, `/cmd_vel`, `/tf`, `/tf_static`, `/gps/fix`, `/imu`, `/sick_scan_2D`, `/joint_states` e status/feedback de NavigateThroughPoses. Registrar também input/output de um futuro conversor Ackermann, distinguindo unidades: o avaliador de steering deve receber yaw-rate de `/cmd_vel_nav`, não sinal já convertido em esterçamento. Plan topics alternativos podem não existir; exigir pelo menos o efetivamente usado e calcular cobertura. `/tf_static` requer QoS transient_local compatível. Costmaps/footprint e contatos são necessários caso o protocolo futuramente inclua restrições/clearance, com custo de disco medido.

Começar recorder antes de enviar goal, confirmar mensagens recebidas, e registrar uma janela que permita diagnóstico sem confundir tempo de missão com espera de startup. O evaluator atual calcula seu cutoff a partir do conteúdo do bag; protocolar janela e timestamps para tornar tempos comparáveis. Depois do resultado/cancelamento, finalizar recorder via SIGINT e aguardar metadata/fechamento antes de extrair. Não confiar em exit code 0 do evaluator original: ele pode gerar BAG_READ_FAILED.

O adapter `tools/auto_tuning/evaluate_trial.py --help` documenta todos os argumentos. Para um dataset prospectivo com protocolo preenchido, o comando deve fornecer explicitamente:

```text
--analysis-repo   checkout congelado 36ba768...
--bag-dir         diretório de UM bag novo finalizado, UM MCAP
--reference-csv   CSV com x_m,y_m,t_ns, associado previamente
--mission-route   YAML efetivamente enviado
--association-manifest JSON com hashes e evidência da associação
--trial-id         identidade prospectiva de trial, igual ao manifest
--attempt-id       identidade exclusiva de tentativa, igual ao manifest
--out-dir         diretório novo fora dos inputs
--scenario-id     1, 2 ou 3 conforme associação validada
--speed-kmh       velocidade nominal congelada
--wheelbase-m     equivalente validado para steering
```

Esta é a especificação de inputs; não são valores fictícios para executar agora. O runbook fornecerá o comando concreto da missão assim que rota/host estiverem definidos. `--preflight-only` valida os inputs sem métricas/ROS; não aprova execução. Saídas `provenance.json`, `evaluator.log`, CSVs e `metrics.json`; métricas indisponíveis permanecem null, com flags e denominadores. Human-likeness não está ativado; energia excluída.

## Gate 1: duas configurações e reset comprovado

Preparação já executável por `prepare_candidates.py`: A usa o YAML completo público, B muda apenas `planner_server.GridBased.non_straight_penalty` de 2,5 para 2,6. Raios físicos permanecem 11,6. Valores são propostas condicionais à versão e Gate 0, não faixas certificadas. Depois do reparo de ambiente, ambos devem usar **o mesmo overlay de localização/BT/cenário congelado**; nenhuma correção entre A e B.

Sequência futura do supervisor, sem intervenção humana entre A/B:

1. Criar diretório exclusivo `trials/candidate_a_run_1`; alocar container novo na mesma imagem imutável, namespace de rede próprio, ROS_DOMAIN_ID definido e saída exclusiva. Sem mounts de estado ROS, volumes de build, cache Gazebo ou bags antigos.
2. Executar os gates de prontidão; registrar configuração pretendida e efetiva; enviar missão com cancelamento e deadlines; gravar feedback/resultados e MCAP.
3. Finalizar recorder, extrair métricas (preferencialmente container de análise separado pela mesma imagem congelada), classificar outcome e preservar logs. Falha técnica não recebe score de navegação.
4. Teardown somente dos recursos desse trial; SIGINT → espera limitada → stop/kill por identidade exata se necessário. Nunca pkill global. Capturar lifecycle/exit/erros, estado Running=false/Pid=0, ausência dos processos/cgroup e endpoints desse container. Sem `reset_world` como substituto de novo processo.
5. Se qualquer verificação de reset falhar: `RESET_CONTAMINATION`, interromper, **não** lançar B. Registrar de modo atômico o estágio final; a retomada cria nova tentativa, nunca sobrescreve A.
6. Criar `candidate_b_run_2` de imagem e inputs congelados; verificar nova época de clock, pose inicial, costmaps/TF/participantes e ausência de estado prévio; repetir ciclo e provar valor 2,6 em vez de 2,5.
7. Produzir relatório A/B com evidências e todos os estados, sem alegar ganho estatístico a partir de dois runs. **Parar para revisão independente**, mesmo com PASS.

Usar namespaces de container + domínio ROS isolado reduz contaminação; domínio diferente sozinho não prova reset. Provar que o container anterior morreu e que o novo começou com mundo/pose/clock/TF/costmaps novos. IDs de imagem, mounts, nomes e ownership de processos fazem parte da evidência.

## Falhas e aceitação

| Classe | Tratamento |
|---|---|
| PREFLIGHT_INVALID | Rejeitar antes de lançar; causa explícita. |
| STARTUP_LIFECYCLE / TF / SENSOR / PLUGIN | Infraestrutura; não atribuir score de navegação. |
| NAVIGATION_INCOMPLETE / HEALTHY_MISSION_TIMEOUT | Outcome válido incompleto/censurado, quando sensores/processos continuam saudáveis; preservar PR/flags. |
| RECORDING / METRIC_FAILURE | Preservar bag/logs; tentar nova extração separada antes de repetir simulação. |
| PROCESS_CRASH | Infraestrutura, distinta de action abort. |
| RESET_CONTAMINATION | Interromper sequência; investigar antes de qualquer próximo trial. |
| INTERRUPTED | Cancelar goal quando possível, finalizar bag, fazer teardown; retomada com nova tentativa e mesma configuração. |

Gate 1 requer **duas execuções reais automáticas**, parâmetros efetivos distintos, resultados/métricas preservados e teardown/reset demonstrados. Testes unitários e YAMLs gerados não bastam. Nesta entrega, o runner fim a fim, readiness automático, resultado de action em JSON e teste real de reset continuam pendentes. Eles dependem da confirmação do bringup e da missão de Gate 0.
