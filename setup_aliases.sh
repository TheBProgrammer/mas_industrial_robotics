#!/bin/bash

# Quick script to setup navigation aliases
# Backs up bashrc just in case something breaks

BASHRC="$HOME/.bashrc"
BACKUP="$HOME/.bashrc.backup_$(date +%Y%m%d_%H%M%S)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}==================================${NC}"
echo -e "${GREEN} Skratch Nav Alias Setup${NC}"
echo -e "${GREEN}==================================${NC}"
echo ""

# backup the bashrc first 
echo -e "${YELLOW}Backing up .bashrc...${NC}"
cp "$BASHRC" "$BACKUP"
echo -e "${GREEN}Backup: $BACKUP${NC}"
echo ""

# clean up old aliases if they exist
if grep -q "# Skratch Navigation Aliases" "$BASHRC"; then
    echo -e "${YELLOW}Found old aliases, cleaning them up...${NC}"
    sed -i '/# Skratch Navigation Aliases/,/# End Skratch Navigation Aliases/d' "$BASHRC"
fi

# add the new aliases
echo -e "${YELLOW}Adding aliases...${NC}"
cat >> "$BASHRC" << 'EOF'

# Skratch Navigation Aliases
alias save_poses='ros2 run skratch_navigation save_poses'
alias navigate='ros2 run skratch_navigation navigate'
alias move_base='ros2 run skratch_navigation navigate'  # shorthand
alias list_poses='cat ~/skratch_ws/src/skratch_navigation/config/navigation_goals.yaml'
alias launch_skratch_sim='ros2 launch skratch_gazebo gazebo.launch.py'
alias localise_skratch='ros2 launch skratch_localization localization.launch.py'
alias enable_skratch_navigation='ros2 launch skratch_navigation nav2.launch.py'
# End Skratch Navigation Aliases
EOF

echo -e "${GREEN}Done!${NC}"
echo ""

# show what we added
echo -e "${GREEN}Added aliases:${NC}"
echo ""
echo "  save_poses                  - save current position"
echo "  navigate [pose]             - go to a saved pose"
echo "  move_base [pose]            - same as navigate"
echo "  list_poses                  - show all saved poses"
echo "  launch_skratch_sim          - start gazebo"
echo "  localise_skratch            - start localization"
echo "  enable_skratch_navigation   - start nav2"
echo ""

echo -e "${YELLOW}To activate: source ~/.bashrc${NC}"
echo ""

# ask if they want to reload now
read -p "Reload .bashrc now? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    source "$BASHRC"
    echo -e "${GREEN}Done - aliases ready to use!${NC}"
else
    echo "Run 'source ~/.bashrc' when you're ready"
fi
echo ""
