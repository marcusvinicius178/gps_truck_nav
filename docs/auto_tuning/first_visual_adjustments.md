# Primeira visualização — ajustes pontuais, 06/11/2026

Marcus confirmou Gazebo Classic 11/RViz no navegador, Arocs renderizado e clock
avançando. O probe candidate20kmh em b078c16 falhou ao abrir a evidência shadow
no host: o processo root do container criou diretório 0700 root:root. Conforme
inspeção informada pelo usuário, o JSON contém SHADOW_COMPLETE, zero publicadores,
nenhuma entrada inesperada, NOT_EXERCISED e cleanup correto. Esta entrega corrige
o acesso; não altera nem transforma o status antigo em aprovação.

## Alterações e limites

- O shadow herda o UID/GID **como vistos dentro do container** do diretório pai
  bind-mounted, criado pelo usuário do host. Transfere a propriedade somente dos
  novos descritores criados exclusivamente. Diretório0700 e arquivos 0600; sem
  recursão, nome de usuário fixo, chmod777 ou execução do probe com sudo. Erros de
  transferência não são ignorados. Caminhos existentes/symlinks não são sobrescritos.
- No modelo há três sensores ray com visualize=1: left_lidar, right_lidar e
  sick_lidar. Cada um desenha seus raios até o alcance configurado. O fonte
  Classic 11.10.2 cria LaserVisual quando esse flag está ativo; ocultá-lo afeta
  a GUI. A sessão cria uma cópia SDF e muda **somente esses três flags** para 0.
  Sempre ativos, update rates, amostras/ranges, ruído, plugins ROS, frames e
  tópicos continuam idênticos. A contagem de raios explica a poluição visual;
  a atribuição exata da captura recebida ainda será confirmada na nova imagem.
- A cópia usa o mesmo alias model:// selecionado, num diretório exclusivo da
  sessão anterior aos caminhos config no GAZEBO_MODEL_PATH. Os modelos instalados
  não contêm esses aliases prospectivos. Meshes continuam resolvidos pelo modelo
  Arocs original. O manifest identifica separadamente fonte/cópia, mudanças e paths.
- A câmera Gazebo passa a uma vista frontal de três quartos orientada pelo yaw
  real do spawn, mirando o centro da carroceria a aproximadamente 20 m. RViz passa
  de distância 45 para 20 m, continua centrado em base_footprint, mantém footprint,
  scan e eixos TF; reduz a opacidade das costmaps e suprime setas/nomes TF
  sobrepostos. O marcador redundante de odometria e o conjunto de trajetórias
  MPPI ficam desmarcados; tópicos de trajetórias continuam preparados.
- A seleção do probe é herdada pelo visualizador: candidate20kmh escolhe seu
  mundo, URDF e YAML Nav2. A cópia gráfica mantém geometria, massas, juntas,
  limites, PID, datum, sensores e parâmetros de navegação. O plugin continua
  no tópico de atuação isolado e o shadow não cria publisher.

Fontes oficiais consultadas: [Scene.cc Classic 11.10.2, ProcessSensorMsg](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/rendering/Scene.cc),
blob 27bf38ff1ead7673b61e9ba7076e9393459a9961, linhas 2246–2268;
[Sensor.cc, Visualize/FillMsg](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/sensors/Sensor.cc),
blob 347b30862debe7c834633105cb56ef57ec6c3048.
Também foram conferidos [RaySensor::IsActive/UpdateImpl](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.10.2/gazebo/sensors/RaySensor.cc),
blob `7e30925c1803b238a40878d35755d88369617a50`, e a assinatura direta no
[gazebo_ros_ray_sensor 3.7.0](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/3.7.0/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp),
blob `73c57e1d6dacd2b51276723f624c1b9dedecf34e`: a publicação ROS recebe o
scan do sensor independentemente do desenho LaserVisual. A execução real ainda
deve confirmar a continuidade dos dados na imagem instalada.

## 1. Novo build e um probe candidate20kmh

No ipt-sim-01, executar este bloco completo em Bash. O SHA exato da entrega está
na resposta da PR; para fixá-lo, usar esse SHA em vez de FETCH_HEAD no worktree
add. O checkout anterior e suas evidências ficam preservados. Não instalar nada
no host, não carregar Humble/Iron no shell e não reiniciar Docker.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
visual_fix_root=$(mktemp -d "$task_root/visual-fix.XXXXXXXX")
printf 'Novas evidências: %s\n' "$visual_fix_root"
(
  set -euo pipefail
  git -C "$task_root/gps_truck_nav" fetch origin engineering/2026-10-preflight-gates \
    2>&1 | tee "$visual_fix_root/git_fetch.log"
  GIT_LFS_SKIP_SMUDGE=1 git -C "$task_root/gps_truck_nav" worktree add --detach \
    "$visual_fix_root/gps_truck_nav" FETCH_HEAD
  cd "$visual_fix_root/gps_truck_nav"
  git lfs checkout 2>&1 | tee "$visual_fix_root/lfs_checkout.log"
  git lfs fsck 2>&1 | tee "$visual_fix_root/lfs_fsck.log"
  git rev-parse HEAD > "$visual_fix_root/checkout_commit.txt"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py build \
    --repo "$visual_fix_root/gps_truck_nav" --output-dir "$visual_fix_root/build"
  /usr/bin/python3 -I tools/auto_tuning/local_phase_a.py probe \
    --repo "$visual_fix_root/gps_truck_nav" --build-dir "$visual_fix_root/build" \
    --output-dir "$visual_fix_root/probe" --navigation-profile candidate20kmh
  /usr/bin/python3 -I - "$visual_fix_root/probe" <<'PY'
from pathlib import Path
import json
import os
import sys
root = Path(sys.argv[1])
status = json.loads((root / 'status.json').read_text())
folder = root / 'actuation_shadow'
summary_path = folder / 'shadow_summary.json'
summary = json.loads(summary_path.read_text())
assert status['status'] == 'OBSERVATIONS_COLLECTED', status
assert status['navigation_profile'] == 'candidate20kmh'
assert summary['status'] == 'SHADOW_COMPLETE', summary
assert summary['command_publishers_created'] == 0
assert summary['unexpected_nonzero_observed'] is False
assert summary['invalid_command_observed'] is False
for path, mode in ((folder, 0o700), (summary_path, 0o600),
                   (folder / 'shadow_commands.jsonl', 0o600)):
    stat = path.stat()
    assert stat.st_uid == os.getuid(), (path, stat.st_uid, os.getuid())
    assert stat.st_mode & 0o777 == mode, (path, oct(stat.st_mode))
print('Evidências shadow legíveis sem sudo; proprietário/modos corretos.')
print('conversion_status:', summary['conversion_status'])
print('diagnostic_status:', status['diagnostic_status'])
print('mission_ready:', status['mission_ready'])
PY
)
```

O helper exige IMAGE_VALIDATED antes do probe. Não mudaram Dockerfile,
dependências APT, pacote C++ ou launches: camadas anteriores são elegíveis ao
cache se a base/digest e o cache local forem iguais. Configuração gráfica e
scripts posteriores ao colcon geram a nova imagem. Não usar --no-cache/prune.
Um novo IMAGE_VALIDATED não altera as imagens antigas.

NOT_EXERCISED é esperado sem comandos; não comprova atuação. A leitura acima é
a regressão real de permissões que falhou anteriormente. Não editar o status
antigo nem usar sudo para repetir o novo probe.

## 2. Abrir Gazebo/RViz

No mesmo terminal, após o bloco anterior concluir:

```bash
(
  set -euo pipefail
  cd "$visual_fix_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py start \
    --repo "$visual_fix_root/gps_truck_nav" --build-dir "$visual_fix_root/build" \
    --probe-dir "$visual_fix_root/probe" --output-dir "$visual_fix_root/visual" --duration 900
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py serve \
    --session-dir "$visual_fix_root/visual" --port 8765 --capture-frames 3
)
```

Abrir no navegador do ipt-sim-01 a URL localhost impressa. Arocs deve ocupar
uma parte legível de cada janela, com terreno/entorno no Gazebo e footprint,
scan, eixos e costmaps no RViz. Os raios azuis extensos devem desaparecer.
Scan vazio/inf continua sendo evidência do sensor; não gerar pontos para preencher
a tela. Planos podem ficar vazios sem goal. O visor não encaminha teclado/mouse.
São preservadas até três capturas PNG para conferir a apresentação.

Ctrl-C no serve encerra a sessão própria. Fechar apenas a aba deixa a sessão
executando até o limite de 900 s. Em outro terminal, consultar/encerrar assim:

```bash
read -r -p 'Caminho completo da pasta visual-fix desta tentativa: ' visual_fix_root
cd "$visual_fix_root/gps_truck_nav"
/usr/bin/python3 -I tools/auto_tuning/visual_observer.py status --session-dir "$visual_fix_root/visual"
/usr/bin/python3 -I tools/auto_tuning/visual_observer.py stop --session-dir "$visual_fix_root/visual"
```

## 3. Retorno necessário

Preservar a pasta impressa. Para conferir só esta correção, retornar:

- build/status.json e probe/status.json;
- probe/actuation_shadow/shadow_summary.json e o resultado da leitura sem sudo;
- visual/status.json, runtime_status.json e presentation_manifest.json;
- visual/visual_captures/, visual/runtime_probe/ e logs da sessão se algo falhar.

A sessão também coleta automaticamente a prontidão existente; não há novos
probes adicionais, serviços de atuação ou missões. O manifest deve indicar
candidate20kmh, alias Arocs candidato e exatamente três flags visualize alterados.
Os hashes da cópia gráfica diferem do fonte por essas alterações e ficam explícitos.

Os testes offline verificam a transferência de identidade com fixture root,
modos/leituras reais do filesystem local como usuário comum, exclusividade,
preservação dos arquivos anteriores, invariância XML fora dos três flags,
câmera, RViz e seleção do candidato. Não executaram Docker/ROS/Gazebo.
A transferência efetiva no bind do ipt-sim-01, a manutenção das mensagens dos
sensores e o novo enquadramento dependem dos passos acima. Sem autorização de
movimento; Gates 0/1 permanecem NOT_YET_VERIFIED.

Resultado desta entrega: **425 testes offline PASS**, auditoria estrutural PASS,
31 blocos Bash/10 Python embutidos com sintaxe verificada e 28 arquivos protegidos
inalterados. Logs e resumo: `evidence/visual_adjustments_tests.log`,
`evidence/visual_adjustments_checks.log` e `evidence/visual_adjustments_summary.json`.
