# Correção pontual — Xauthority do GDM com número vazio

Base: `3911c697bf270a4adc4b05e1fd7dcb15d38889f5`, PR #2 DRAFT.
Escopo: seleção da credencial em `tools/auto_tuning/desktop_display.py`,
função `authority_cookie()`, e contagens não secretas em `inspect_host()`.

## Causa e regra de seleção

O host informou X11, `DISPLAY=:1`, hostname `ipt-sim-01` e autoridade
`/run/user/1001/gdm/Xauthority`. Os dois registros MIT de 16 bytes tinham
número vazio: um Local com hostname correspondente e um Wild. A implementação
anterior exigia número exatamente igual a `1` e excluía ambos. O relato não
estabelece se os dois segredos do host são iguais; o diagnóstico corrigido fará
essa comparação somente em memória.

O libXau aceita um número vazio no registro como correspondência de qualquer
número consultado. Local exige endereço correspondente; Wild dispensa a
correspondência de família/endereço, mas não ignora um número explícito
incompatível. O GDM escreve o mesmo cookie duas vezes, alterando Local para
Wild, sem preencher o número. Fontes primárias consultadas:

- [GDM, `prepare_auth_file()`](https://github.com/GNOME/gdm/blob/main/daemon/gdm-x-session.c),
  blob de código `ed01fbe5264ad2de8e30276cf97d6dd1824bed29`.
- [libXau, `XauGetAuthByAddr()`](https://github.com/NetBSD/xsrc/blob/trunk/external/mit/libXau/dist/AuGetAddr.c),
  blob de código `6de2be300ddeb727852401d5f065ca6ef6acf3c6`.
- [libXau, `XauGetBestAuthByAddr()`](https://github.com/NetBSD/xsrc/blob/trunk/external/mit/libXau/dist/AuGetBest.c),
  blob de código `50eba63a34bf70946d98009788ca6676a7be1a9d`.

A aplicação mantém uma política mais restrita que a escolha do primeiro
registro pelo libXau: exige exatamente um segredo distinto entre todos os
registros elegíveis, independentemente da ordem e de aparente especificidade.
Duplicatas idênticas são aceitas. Segredos distintos, inclusive entre número
exato e vazio, geram `XAUTHORITY_AMBIGUOUS_CREDENTIALS` antes de abrir o display.
Nenhum segredo conflitante é escolhido ou experimentado por prioridade.

O protocolo MIT, tamanho de 16 bytes, arquivo privado do usuário, leitura
limitada, rejeição de dados truncados e display local explícito continuam
obrigatórios. As evidências acrescentam somente contagens: registros elegíveis,
números vazios/exatos, famílias Local/Wild, credenciais distintas e duplicatas.
Não registram cookies, codificações, hashes ou impressões digitais de segredos.

## Autenticação e isolamento preservados

Selecionar o registro não produz PASS. O guardião fornece a credencial
explicitamente ao Xlib e abre o display selecionado; exige controle de acesso
ativo e XSECURITY. Cria uma credencial nova e diferente, testa uma conexão com
ela e GLX, e exige revogação e remoção ao encerrar. Essa sequência não foi
alterada. Uma conexão X11 isolada não comprova exclusivamente a posse do cookie
quando o servidor tem exceções ACL; por isso ela não é usada para desempatar
segredos. A credencial original do GDM nunca é montada no container.

Nenhuma mudança em Xvfb, Docker, supervisão, R1, autoridade de comandos,
watchdogs, contenção, planta, sensores, Nav2 ou visualização. Nenhum processo
ROS, Docker, simulador ou display real foi iniciado nesta validação offline.

## Validação offline

- Suíte completa: **684 testes aprovados, zero falhas, erros ou skips**.
- 16 regressões novas em `tests/test_desktop_gdm_authority.py`: formato real
  do GDM em ambas as ordens, duplicatas e segredos distintos, Local/Wild,
  números vazios/exatos/incompatíveis, protocolo/tamanho estritos, registros
  truncados e ausência de segredos nas evidências.
- Comparação executada com `libXau.so.6`, chamando `XauGetAuthByAddr()` e
  `XauGetBestAuthByAddr()` sobre arquivos temporários sintéticos, sem X server.
- Integração do parser e guardião com backend X11 simulado: display `:1`,
  exportação somente do novo cookie, limpeza/revogação e rejeição de
  autenticação, controle de acesso, SECURITY ou GLX indisponíveis.
- Classes de autenticação/sessão e guardião comparados com a base: inalterados.

Resultados: `evidence/gdm_xauthority_unit_tests.log` e
`evidence/gdm_xauthority_offline_summary.json`. Os resultados anteriores foram
preservados. Autenticação real no ipt-sim-01 ainda não foi validada.

## Única próxima etapa no host

No checkout corrigido, em terminal da sessão gráfica local do usuário Marcus,
sem sudo e sem sobrescrever DISPLAY/XAUTHORITY, executar somente:

```bash
(
  set -euo pipefail
  gdm_diagnostic_root=$(mktemp -d "$HOME/nav2-gdm-diagnostic.XXXXXXXX")
  printf 'Evidências desta tentativa: %s\n' "$gdm_diagnostic_root"
  /usr/bin/python3 -I tools/auto_tuning/desktop_display.py diagnose \
    --output-dir "$gdm_diagnostic_root/display-diagnostic" \
    2>&1 | tee "$gdm_diagnostic_root/diagnose-console.log"
)
```

Exigir `DISPLAY_DIAGNOSTIC_PASS`, conexão com a credencial temporária e GLX
confirmados, revogação/remoção confirmadas e retorno zero do guardião. Para o
par GDM informado, se os cookies forem iguais, esperar `matching_records=2`,
`empty_number_records=2`, `unique_credentials=1`, `duplicate_records=1`.
Se diferentes, a rejeição é o resultado seguro esperado, sem Docker.

Preservar `display_diagnostic.json`, `display_guardian.log` e o console desta
tentativa. Não copiar o arquivo Xauthority nem a credencial temporária para as
evidências. **Aguardar esse resultado antes de fornecer/executar a próxima
sequência de build, regressões e janelas interativas.** Esta etapa não exige
build, malhas LFS ou modificação da imagem. Gate 0 e Gate 1 continuam
`NOT_YET_VERIFIED`; não há autorização de movimento.
