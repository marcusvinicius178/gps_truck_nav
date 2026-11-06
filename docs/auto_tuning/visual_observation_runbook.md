# Gazebo Classic e RViz — visualização ao vivo, somente observação

Este modo abre os aplicativos reais dentro de uma sessão Xvfb privada e mostra seus pixels no navegador do `ipt-sim-01`, aproximadamente uma vez por segundo. Gazebo fica à esquerda; RViz, à direita. **Não existe canal de teclado/mouse, endpoint de goal, controle de modelo ou comando de movimento no visor.** Isso evita chamar de read-only um gzclient nativo, que normalmente oferece ferramentas capazes de alterar a simulação.

**Atualização de 06/11/2026:** Marcus confirmou a primeira renderização real de
Gazebo/RViz com Arocs e clock avançando. Para repetir somente o necessário com
as correções de acesso/enquadramento e `candidate20kmh`, seguir o procedimento
completo em [first_visual_adjustments.md](first_visual_adjustments.md). As seções
abaixo conservam a interface geral dos comandos; a seleção é herdada do probe.

O primeiro modo não oferece pan/zoom interativo. A câmera Gazebo é posicionada sobre o spawn do Arocs; RViz acompanha `base_footprint`. São mostrados modelo, frames, pose, scan, footprint e costmaps. Displays de trajetórias estão preparados, mas ficam vazios antes de planejamento autorizado; não enviar um objetivo apenas para preencher a tela. O MPPI 1.2.10 publica `MarkerArray` em `/trajectories` e `Path` em `/transformed_global_plan`, quando seu visualizador é usado; esta revisão não muda `visualize=false` para gerar dados. `/plan` é o caminho global. O scan pode não ter retornos finitos no cenário atual, e ausência de pontos não comprova falha nem sucesso de detecção.

## Isolamento e limites

- Usa **a mesma imagem por ID, fonte e perfil** do build/probe concluídos, incluindo `candidate20kmh`. No candidato, seleciona seu mundo, URDF e configuração Nav2, além do adaptador shadow sem saída de atuação.
- Container próprio com `network=none`, limites de CPU/RAM existentes, sem portas Docker, display do host, GPU, dispositivos, privilégios, socket Docker, host networking ou mounts de outros ambientes. Nenhum pacote é instalado no host ou acrescentado à imagem para esse modo.
- HTTP é servido por Python stdlib **somente em 127.0.0.1**, com caminhos exatos e GET de página/imagem. Não encaminha entrada para X11/ROS/Gazebo. A URL de sessão é impressa pelo helper.
- O mundo visual acrescenta `<gui><camera>` numa cópia da tentativa. Outra cópia, em `presentation_models/`, muda somente `visualize` de 1 para 0 nos três sensores `ray`, ocultando os desenhos dos raios. Os sensores continuam habilitados; ranges, amostras, ruído, update rates, plugins e tópicos são preservados. `presentation_manifest.json` distingue os hashes do modelo fonte e da cópia efetivamente selecionada pela sessão. Originais, geometria, spawn, datum, física, BTs e parâmetros de navegação permanecem intactos.
- O backend visual continua executando física e sensores, como o probe. O modo não congela a planta nem comprova imobilidade física; **nenhum objetivo ou comando de atuação é enviado pelo helper**. Não equivale a uma fronteira de segurança contra alguém que já possua acesso administrativo ao Docker.
- Duração padrão 900 segundos, máximo 1800. Timeout, saída de aplicativo obrigatório ou sinal encerram a sessão. Containers e imagens anteriores são preservados. `Ctrl-C` em `serve` para somente o container identificado desta sessão; fechar a aba do navegador, sozinho, não para o container, que permanece sujeito ao prazo.

Pré-requisitos da imagem: Xvfb, gzclient, rviz2 e libX11, além do build `IMAGE_VALIDATED`. O smoke só verifica disponibilidade, sem abrir GUI. A sessão exige também `OBSERVATIONS_COLLECTED` de um probe da **mesma fonte/imagem**. A coleta pode continuar PARTIAL ou conter bloqueios conhecidos; isso não autoriza missão. A execução real da GUI permanece uma verificação local, não realizada nos testes offline.

## 1. Iniciar a sessão após build/probe

**Host e diretório:** terminal do `ipt-sim-01`; usar o checkout dedicado. No mesmo terminal da seção 2.4 do runbook principal, `review3_root` já identifica a nova tentativa. Em outro terminal, o bloco pede esse caminho exato. Não apontar para uma imagem antiga ou editar seus manifests.

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
if [[ -z "${review3_root:-}" ]]; then
  read -r -p 'Caminho completo da terceira revisão com build/probe concluídos: ' review3_root
fi
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  test -d "$review3_root"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py start \
    --repo "$task_root/gps_truck_nav" \
    --build-dir "$review3_root/build" \
    --probe-dir "$review3_root/probe" \
    --output-dir "$review3_root/visual" \
    --duration 900
)
```

O start aguarda até 60 segundos pela identificação das duas janelas e de um framebuffer válido. Se falhar, mantém o diagnóstico e tenta parar somente seu container. Não transformar falha de janela ou tela vazia em PASS apenas porque o processo existe. O helper dentro do container também inicia um novo `probe_readiness.py` de 30 segundos após aquecimento; esses JSONs pertencem à sessão visual e são independentes do probe anterior.

## 2. Abrir o visor

No mesmo terminal, executar:

```bash
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py serve \
    --session-dir "$review3_root/visual" \
    --port 8765
)
```

Abrir no navegador **desse host** a URL `http://127.0.0.1:8765/.../` impressa. A câmera já está definida; cliques na imagem não são encaminhados. A atualização de ~1 Hz e o framebuffer copiado durante rendering podem produzir tearing/atraso visual. Não usar essa imagem como medição sincronizada de latência, velocidade, trajetória ou erro; usar os timestamps ROS e evidências estruturadas.

Se a porta estiver ocupada, a tentativa de serve falha e encerra somente a sessão visual própria; não mata outro servidor. Use uma nova sessão e uma porta livre explícita. Não mudar o bind para 0.0.0.0 nem expor esse visor na rede. Não é necessário `xhost +`, copiar cookies Xauthority, instalar RViz no Humble ou reiniciar Docker.

Capturas PNG não são salvas por padrão. Para preservar no máximo três imagens desta sessão, usar este bloco **em vez do anterior**, uma única vez:

```bash
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py serve \
    --session-dir "$review3_root/visual" \
    --port 8765 \
    --capture-frames 3
)
```

As capturas ficam em `visual/visual_captures/`, cada uma com SHA-256 e horário UTC. O limite aceito é 0–10. Elas documentam a apresentação, não precisão geométrica ou consumo de parâmetros.

## 3. Consultar ou interromper

Em outro terminal, consultar o estado com o bloco completo:

```bash
task_root="$HOME/nav2-phase-a-ipt-sim-01.pMbd0n06"
read -r -p 'Caminho completo da terceira revisão desta sessão: ' review3_root
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py status \
    --session-dir "$review3_root/visual"
)
```

Para encerrar sem depender do navegador, no terminal em que essas variáveis estão definidas:

```bash
(
  set -euo pipefail
  cd "$task_root/gps_truck_nav"
  /usr/bin/python3 -I tools/auto_tuning/visual_observer.py stop \
    --session-dir "$review3_root/visual"
)
```

`stop` confere ID, imagem, labels e isolamento antes da ação; não escolhe containers pelo nome do processo nem para outros projetos. Registra `Running=false`, `Pid=0` e preserva o container parado. Cada consulta/cleanup cria um diretório próprio em `visual/operations/`; não apaga logs anteriores. Para reiniciar, escolher **outro output-dir** em nova tentativa; não reutilizar `visual` nem apagar seus arquivos.

## 4. Evidências e critério de aceitação visual

Enviar `visual/status.json`, `runtime_status.json`, `presentation_manifest.json`, `session_commands.jsonl`, os logs dos aplicativos, `runtime_probe/` e `operations/`. Incluir capturas somente se escolhidas. O framebuffer XWD é um arquivo de trabalho que pode conter cerca de 8 MB; não precisa ir no pacote.

Após parar a sessão, este bloco gera um ZIP exclusivo apenas das evidências visuais:

```bash
/usr/bin/python3 -I - "$review3_root/visual" <<'PY'
from pathlib import Path
import hashlib
import sys
import zipfile
root = Path(sys.argv[1]).resolve(strict=True)
archive = root.parent / 'nav2_phase_a_visual_observation_evidence.zip'
allowed = {'.json', '.jsonl', '.log', '.world', '.png'}
with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as output:
    for path in sorted(root.rglob('*')):
        if (path.is_file() and not path.is_symlink()
                and path.resolve().is_relative_to(root)
                and (path.suffix in allowed or path.name == 'container.id')):
            output.write(path, Path('visual') / path.relative_to(root))
print(archive)
print('SHA-256:', hashlib.sha256(archive.read_bytes()).hexdigest())
PY
```

Aceitar somente o escopo de apresentação quando: ambas as janelas forem observadas, Arocs/meshes aparecerem, displays relevantes tiverem dados/TF coerentes, a física/clock continuarem observáveis, a coleta registrar os bloqueios reais, não houver canal de entrada no visor, e o cleanup identificar/parar a sessão correta. Tela vazia, erros de mesh/OpenGL/TF ou processo encerrado exigem preservar logs e investigação; não instalar drivers nem retunar sensores para mascará-los.

O viewer deve recusar frames com heartbeat de sessão ausente, parado ou vencido; esse heartbeat mede vida da apresentação, não substitui `/clock`. A primeira renderização foi confirmada pelo usuário; o novo enquadramento, a ocultação dos raios e o acesso às evidências precisam da repetição no host. **Gate 0 e Gate 1 continuam NOT_YET_VERIFIED.** Seleção interativa de objetivo, cancelamento de ação e parada física serão habilitados somente em uma etapa posterior autorizada, após validação da atuação.

Referências primárias: [Xvfb Jammy, framebuffer XWD](https://manpages.ubuntu.com/manpages/jammy/man1/Xvfb.1.html), [formato produzido por Xvfb](https://github.com/XQuartz/xorg-server/blob/master/hw/vfb/InitOutput.c), [GUI/câmera SDF 1.6](https://github.com/gazebosim/sdformat/blob/sdf9/sdf/1.6/gui.sdf), [tópicos de visualização MPPI 1.2.10](https://github.com/ros-navigation/navigation2/blob/1.2.10/nav2_mppi_controller/src/trajectory_visualizer.cpp). A presença desses mecanismos no source não substitui a execução e inspeção da imagem instalada.
