---
img_name: img/populating_{}.png
template: ./populate.simple_template
type: html
board: ../hw/kicad_export/RBCX.kicad_pcb
libs: ./PcbDraw_RBCX_Eagle-export
params:
    - --dpi 200
    - --style builtin:jlcpcb-green-hasl.json
...

# RBCX - návod na osazení pro Robotku

- [[front| ]] Vrchní strana DPS, na které se nachází většina SMD součástek. Zde se bude osazovat jen USB-C konektor.
- [[back | ]] Spodní strana DPS, na kterou budete osazovat většinu součástek v tomto návodě
- [[back | P1, P2 ]] Začneme osazením konektorů pro motory.
- [[back | JP6, JP13, JP18, JP29, JP40 ]] Osadíme pinheady 2x4 podle obrázku. Pájíme na pozice U1 a U2, 1 a 2, UART 3V3 a SEL. Pinheady se vždy zasouvají **kratším koncem do desky**.
- [[back | JP22, JP25, JP28, JP30 ]] Osadíme pinheady 2x7 podle obrázku.
- [[back | JP8, JP12, JP15, JP19 ]] Osadíme pinheady 2x9 podle obrázku.
- [[back | JP31 ]] Osadíme pinheady 1x3 podle obrázku. Pájíme na pozici iLED.
- [[back | IM9 ]] Osadíme dutinkové lišty 1x19 pro ESP32 mikrokontrolér.
- [[back | B1 ]] Osadíme PIEZO podle obrázku. V tomto případě na polaritě nezáleží. Neodstraňujte kryt! Kryt se odstraní až po dokončení a umytí desky.
- [[back | X1 ]] Osadíme svorkovnici na pozici B+, B- a Bmid. Otvory musí směřovat ven z desky.
- [[back | C1, C2 ]] Osadíme elektrolytické kondenzátory podle obrázku. POZOR u těchto kodenzátorů záleží na polaritě! Na obalu kondenzátoru i na desce je polarizace vyznačena. Před pájením přijde KAŽDÝ na kontrolu za vedoucím.
- [[back | IM1 ]] Osadíme nachystaný spínaný zdroj podle obrázku. Dávejte pozor na otočení. Součástky spínaného zdroje musí směřovat vlevo podle obrázku.
- [[back | ]] Kompletní osazení desky pro Robotku.
