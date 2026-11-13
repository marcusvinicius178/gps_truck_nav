# Auditoria do pipeline de métricas — 29/10/2026

## Evidência e limite

**VERIFIED BY STATIC INSPECTION:** `nav2_paper_scripts` permanece em `36ba76823cc7a124540412cf3ce09c73e6cd1d05`; nenhuma alteração no repositório de análise, nenhum bag histórico reprocessado. Os achados de 14/10 continuam válidos. O ambiente não contém ROS 2/MCAP; não houve extração de métricas reais novas.

**VERIFIED BY EXECUTION:** 22 testes de contrato do novo adaptador passaram em Python sem ROS; `--help` funciona. Esses testes usam fixtures artificiais identificadas como tal, não simulam nem comprovam Gates 0/1.

**PROPOSED BUT NOT TESTED:** execução do adaptador sobre bag prospectivo real dentro de Iron. README da análise (linhas 59–66) informa desenvolvimento das ferramentas de bag com **Jazzy**. Compatibilidade das APIs usadas com o Iron escolhido precisa ser exercitada; a análise não autoriza migrar a simulação para Jazzy.

## Entradas, dependências e semântica

| Item | Evidência na revisão fixada | Consequência |
|---|---|---|
| Evaluator principal | `scripts/simulation/eval_nav2_one_combination.py:169–240` | CLI requer runs-dir, scenario-id, speed-kmh, planner-id, controller-id, gt-csv; out-dir deve ser explícito. |
| Dependências reais | imports nas linhas 95–101; requirements_v3.txt:2–8 | Para core: NumPy, `rosbag2_py`, `rclpy.serialization`, `rosidl_runtime_py.utilities`, tipos ROS registrados e storage MCAP. Bibliotecas ROS vêm do ROS sourced, não de pip. Matplotlib/PyYAML constam nas dependências gerais, não são imports diretos do core principal. |
| Backend | linhas 650–668 | `SequentialReader`, `StorageOptions(storage_id='mcap')`, CDR. Verificar `ros-iron-rosbag2-storage-mcap` e `ros-iron-rosbag2-py` no ambiente efetivo. O Dockerfile não declara MCAP explicitamente. |
| Descoberta dos bags | linhas 402–424 | Busca recursiva de `metadata.yaml`. Se múltiplos MCAPs, escolhe apenas `_0.mcap` ou o primeiro. O adaptador rejeita split bag para evitar resultado truncado silencioso. |
| Nome | linhas 427–431 | Run ID são dígitos finais do nome, ou 0. Recomendar `candidate_a_run_1`, `candidate_b_run_2`; nome não prova identidade, registrar IDs no manifest. |
| Trajetória executada | linhas 202, 665–667, 765–766 | `/odometry/global`, mínimo de duas amostras, posteriormente remove posições consecutivas duplicadas. Odometry twist.linear.x é a fonte usual de velocidade. |
| Planos | linhas 203–211, 895–907 | Prioridade `/plan`, `/received_global_plan`, `/transformed_global_plan`, `/plan_smoothed`. Plano ativo = último anterior/igual ao timestamp de armazenamento MCAP da odometria. |
| TF | linhas 674–675, 1288–1403 | `/tf`, `/tf_static` para transformação direta/inversa dos frames. Não presumir solução genérica de árvore TF multihop. Preferir odometria e plano no mesmo frame, mas confirmar mensagens. |
| Comando | linhas 213, 1435–1467 | Prioridade `/cmd_vel_nav`, `/cmd_vel`; necessário para steering-rate reconstruído. Ausência deixa métricas indisponíveis. |
| Referência de progresso | linhas 937–975, 1976–1989 | `--gt-csv` exige `x_m,y_m,t_ns`; pelo menos dois pontos distintos. Referência é alinhada à posição/direção da execução; não é o plano Nav2 para RMSE lateral. |
| Thresholds | linhas 228–238, 1157–1213, 2008–2013 | Sucesso geométrico PR≥0,90; valid_for_mission PR≥0,70; tracking depende também de plano com ≥2 amostras e cobertura ≥0,80. Flags devem ser preservadas. |
| Cutoff | linhas 1180–1192 | Primeiro PR≥0,90 se atingido; caso contrário, instante de máximo progresso. Tempo de falha não é tempo de conclusão comparável. |
| Jerk | linhas 1420–1431 | Segunda derivada de velocidade com médias móveis de 11 amostras antes de cada derivada. Sem energia, potência ou conforto humano medido. |
| Steering | linhas 215–225, 1444–1462 | `atan(L_eq*wz/vx)`, filtro |vx|≥0,50 m/s. **Default L_eq=6,804 m**; não confundir automaticamente com entre-eixos físico 5,950 m de outro contexto. Adaptador exige valor explícito validado. |
| Humanos | linhas 1471–1498, 2063–2074 | Camada opcional: raw GPS dos humanos + waypoint de origem; `--gt-gps-csv-glob` e `--waypoint-file` devem ser fornecidos juntos. |
| Erro mascarado | linhas 1901–1961, 2214 | Falha ao ler bag gera linha com PR=0, `bag_read_failed`, NaNs e continua; script pode retornar 0. Adaptador rejeita isso como **MEASUREMENT_FAILURE**, jamais como desempenho ruim do candidato. |

A agregação limita smoothness e demais métricas contínuas à população `valid_for_mission` (linhas 1705–1772). O CSV pode conter valores de smoothness em tentativa não elegível; preserve a flag e não use esses valores indiscriminadamente como objetivo.

## Referências disponíveis e lacunas

Há seis CSVs locais em `data/human_reference/gt_mean_references/`: scenario1/2/3 × vel15/20. São suficientes como entradas XY, **se** a associação entre missão e referência for verificada. O pipeline suplementar espera também `scenario1_waypoints_debug.yaml`, `scenario2_waypoints_debug.yaml`, `scenario3_waypoints_debug.yaml` e raw-GPS por bag (`run_supplementary_tables_pipeline.py:44–64,390–414`). Esses artefatos de waypoints e raw-GPS não estão no checkout auditado. Não são necessários para as métricas core quando human-likeness estiver desativado; não inventar a camada humana.

A missão ativa pública em `truck_bringup/params/empty_world_gps_wps.yaml:112–153` contém dez pontos sob comentário **SCENARIO 5_6 / Straight line + ARC 40**; os segmentos anteriores estão comentados. Isso NÃO autoriza associá-la automaticamente ao CSV S1, nem garante a numeração final S3. A correspondência de forma, extensão e condição deve ser revisada antes de calcular PR. Um teste curto recortado precisa de referência e alinhamento/cutoff adequados declarados previamente; usar a referência inteira arbitrariamente distorce progresso. Para este adaptador ficam permitidos apenas IDs 1, 2 ou 3, em associação revisada.

`logged_waypoint_follower.py:80–82` preserva translação map-x `+4.8025 m`; não é um lever arm rotacionado. O início e a rota convertida via fromLL precisam ser observados e congelados. O sender não tem prazo global de espera por fromLL/Nav2 ou missão (linhas 88–109) e retorna status textual/exit code, não a avaliação geométrica. A implementação do supervisor deverá tratar isso separadamente.

## Novo adaptador

Arquivos completos adicionados ao repositório de integração:

- `tools/auto_tuning/evaluate_trial.py`
- `tests/test_evaluate_trial.py`

O adaptador fixa o commit de análise, compara o hash do evaluator no worktree com o conteúdo Git original, aceita um único bag finalizado de um único MCAP, exige saída nova fora do bag e do repositório de análise, valida CSV finito, mantém flags/nulls, salva logs, hashes e resultado JSON. Não modifica dados de entrada. Também verifica que bag, referência, rota, associação e evaluator não mudaram durante a extração.

Argumentos obrigatórios: `--analysis-repo`, `--bag-dir`, `--reference-csv`, `--mission-route`, `--association-manifest`, `--trial-id`, `--attempt-id`, `--out-dir`, `--scenario-id`, `--speed-kmh`, `--wheelbase-m`. `--timeout-seconds` default 300; `--preflight-only` verifica contratos estáticos sem invocar evaluator nem criar saída.

Manifest de associação requerido (schema, **não preencher com alegação fictícia de revisão**):

```json
{
  "schema_version": 2,
  "dataset_kind": "prospective_auto_tuning",
  "trial_id": "candidate_a",
  "attempt_id": "attempt_1",
  "scenario_id": 1,
  "speed_kmh": 20,
  "reference_sha256": "SHA256_DO_CSV_REAL",
  "mission_route_sha256": "SHA256_DA_ROTA_REAL",
  "bag_metadata_sha256": "SHA256_DO_METADATA_YAML_FINALIZADO",
  "bag_mcap_sha256": "SHA256_DO_MCAP_FINALIZADO",
  "association_evidence": "Descrição e localização da revisão que confirma a relação entre esta missão e esta referência."
}
```

O manifest precisa corresponder aos dois IDs fornecidos na CLI e aos SHA256 reais do metadata.yaml e MCAP finalizados. Isso impede reutilizar acidentalmente um manifest para outro bag/tentativa; a origem prospectiva ainda é declaração do supervisor/operador, não autenticação independente. Hashes e texto são uma declaração de proveniência, **não uma prova automática de equivalência geométrica**; o resultado registra esse limite explicitamente. O script não cria esse manifest sozinho nem escolhe S1 como default. O código só deve processar dados novos cuja origem prospectiva foi confirmada pelo supervisor/operador.

Saídas por tentativa: `provenance.json`, `evaluator.log`, `evaluator/per_run_metrics.csv` e demais saídas originais, `metrics.json`. Estado `METRICS_EXTRACTED` significa extração disponível; não significa Nav2 action SUCCEEDED, navegação segura nem Gate PASS. `action_status=null`, `engineering_gate_status=NOT_ASSESSED` sempre. Status técnicos incluem `PREFLIGHT_REJECTED`, `MEASUREMENT_FAILURE`, `INTERRUPTED`; exit 0/2/3/130 respectivamente (0 também para preflight-only identificado como tal).

## Teste executado

Diretório: raiz do checkout `gps_truck_nav`. Pré-requisito: Python ≥3.9 (Iron/Ubuntu 22.04 usa Python 3.10).

```bash
python3 -m unittest discover -s tests -p test_evaluate_trial.py -v
python3 tools/auto_tuning/evaluate_trial.py --help
```

Resultado: **22/22 PASS**. Cobertura: bag dividido/nested/não finalizado, referência degenerada/não finita, associação/digest/dataset errado, bag_read_failed com retorno nominal, PR não finito, thresholds contraditórios, tracking ausente com nulls, tentativa incompleta válida, bag errado, múltiplas linhas e argumentos congelados. As regressões adicionais cobrem JSON não objeto, CSV truncado/excedente, ausência/divergência dos IDs, troca de metadata/MCAP, PR>1 (tolerância exclusiva de arredondamento 1e-12), tempo útil superior à duração total, RMSE/RMS negativos e tracking com apenas uma amostra. A exclusão de tracking usa a contagem exportada e cobertura, não apenas has_plan. A CLI devolve PREFLIGHT_REJECTED sem traceback para entradas malformadas. Não houve teste de leitura MCAP/ROS, geração de métricas reais ou end-to-end.

## Menor rota crível para Gate 0/1

1. Confirmar ambiente e recorder MCAP; congelar versões.
2. Validar uma associação missão–referência e o equivalente geométrico usado no steering; salvar a associação de rota/referência com evidência antes do bag novo; após finalizar a gravação, concluir o manifest da tentativa com os IDs e hashes reais do bag.
3. Registrar um único MCAP sem split, finalizando via SIGINT antes de extrair. Registrar `/clock`, `/odometry/global`, `/plan`, `/received_global_plan`, `/transformed_global_plan`, `/plan_smoothed`, `/cmd_vel_nav`, `/cmd_vel`, `/tf`, `/tf_static`, `/gps/fix`, `/imu`, status/feedback de ação e parâmetros efetivos conforme grafo observado. `/clock` e sensores adicionais apoiam diagnóstico; o evaluator core consome apenas o subconjunto descrito na tabela.
4. Executar adaptador em nova saída. Verificar logs, flags de elegibilidade e mensagem real de conclusão de ação separadamente.
5. Repetir automaticamente após teardown/reset comprovado, com outro candidato. Só então avaliar Gate 1; os testes unitários acima não substituem a execução.
