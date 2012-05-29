##### left screen
gnome-terminal --geometry 72x26+0+0 		-x bash -c "ssh -X ${1}; bash" --working-directory=$(pwd) &
gnome-terminal --geometry 82x26+605+0		-x bash -c "ssh -X ${1}; bash" --working-directory=$(pwd) &
gnome-terminal --geometry 72x25+0+526		-x bash -c "ssh -X ${1}; bash" --working-directory=$(pwd) &
gnome-terminal --geometry 82x25+605+526		-x bash -c "ssh -X ${1}; bash" --working-directory=$(pwd) &
