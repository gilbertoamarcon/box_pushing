#!/usr/bin/env bash
matlab -nodesktop -nodisplay -r 'generate_img_seq('$1')' > /dev/null
matlab -nodesktop -nodisplay -r 'generate_gif_clip('$1','$2')' > /dev/null