#!/usr/bin/env python3

import re
import sys
from typing import Optional, Tuple


def map_changelog_type(pr_type: str) -> str:
    """Map PR changelog type to Valhalla subsection name."""
    type_mapping = {
        'ADDED': 'Enhancement',
        'CHANGED': 'Enhancement',
        'UPDATED': 'Enhancement',
        'FIXED': 'Bug Fix',
        'REMOVED': 'Removed',
    }
    return type_mapping.get(pr_type.upper(), 'Enhancement')


def extract_changelog_from_pr(
    pr_body: str, pr_number: str, pr_url: str
) -> Optional[Tuple[str, str, str]]:
    """Extract changelog entry from PR body in Valhalla format."""

    pr_body = pr_body.replace('\r\n', '\n').replace('\r', '\n').strip()

    # Look for changelog section in PR body
    changelog_pattern = r'(?i)##+\s*changelog\s*\n+([^#]+)'
    match = re.search(changelog_pattern, pr_body, re.DOTALL | re.IGNORECASE)

    if not match:
        print(f"Debug: No changelog pattern found in PR body. PR body content: '{pr_body}'")
        return None

    changelog_content = match.group(1).strip()
    print(f"Debug: Found changelog content: '{changelog_content}'")

    # Extract changelog type and entry (more flexible pattern)
    type_pattern = r'(?i)^\s*(added|changed|fixed|removed)[:\s]+(.+)$'
    type_match = re.search(type_pattern, changelog_content)

    if not type_match:
        print(f"Debug: No valid changelog type found in: '{changelog_content}'")
        return None

    pr_changelog_type = type_match.group(1).upper()
    changelog_entry = type_match.group(2).strip()

    # Map to Valhalla subsection
    valhalla_subsection = map_changelog_type(pr_changelog_type)

    # Format the entry in Valhalla style
    formatted_entry = f'{pr_changelog_type}: {changelog_entry} [#{pr_number}]({pr_url})'

    return formatted_entry, valhalla_subsection, pr_changelog_type


def update_changelog_file(changelog_entry: str, subsection: str, changelog_path: str = 'CHANGELOG.md'):
    """Update the changelog file with the new entry."""
    try:
        with open(changelog_path, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: {changelog_path} not found")
        return False

    # Find UNRELEASED section
    unreleased_pattern = r'(## UNRELEASED\n)(.*?)(?=## Release Date|$)'
    match = re.search(unreleased_pattern, content, re.DOTALL)

    if not match:
        print("UNRELEASED section not found in changelog")
        return False

    unreleased_section = match.group(2)

    # Check if the subsection exists
    subsection_pattern = rf'\*\*{re.escape(subsection)}\*\*'
    subsection_match = re.search(subsection_pattern, unreleased_section, re.IGNORECASE)

    if not subsection_match:
        print(f"Subsection '{subsection}' not found in UNRELEASED section. No changes made.")
        return False

    # Add entry to existing subsection - find the last entry and append after it
    # Pattern to find the subsection and all its entries
    subsection_full_pattern = rf'(\*\*{re.escape(subsection)}\*\*\n)((   \* .*\n)+)'
    subsection_full_match = re.search(subsection_full_pattern, unreleased_section, re.IGNORECASE)

    if subsection_full_match:
        # Found the subsection with entries, append to the end
        subsection_entries = subsection_full_match.group(2)

        # Add new entry at the end of existing entries
        new_subsection_entries = subsection_entries.rstrip() + f'\n   * {changelog_entry}\n'
        new_section = unreleased_section.replace(subsection_entries, new_subsection_entries)
    else:
        # Subsection exists but has no entries yet
        new_section = re.sub(
            rf'(\*\*{re.escape(subsection)}\*\*\n)',
            rf'\1   * {changelog_entry}\n',
            unreleased_section,
            flags=re.IGNORECASE,
        )

    # Replace the section
    new_content = content.replace(unreleased_section, new_section)

    # Write back to file
    try:
        with open(changelog_path, 'w') as f:
            f.write(new_content)
        return True
    except Exception as e:
        print(f"Error writing to {changelog_path}: {e}")
        return False


def main():
    if len(sys.argv) != 4:
        print("Usage: python update_changelog.py <pr_body> <pr_number> <pr_url>")
        sys.exit(1)

    pr_body = sys.argv[1]
    pr_number = sys.argv[2]
    pr_url = sys.argv[3]

    print(f"Debug: PR Body received: '{pr_body}'")
    print(f"Debug: PR Number: {pr_number}")
    print(f"Debug: PR URL: {pr_url}")

    result = extract_changelog_from_pr(pr_body, pr_number, pr_url)

    if not result:
        print("No valid changelog entry found in PR body")
        sys.exit(0)

    changelog_entry, subsection, pr_type = result
    print(f"Detected PR type: {pr_type} -> Subsection: {subsection}")

    if update_changelog_file(changelog_entry, subsection):
        print(f"Successfully updated changelog with: {changelog_entry}")
        print(f"Added to subsection: {subsection}")
    else:
        print("No changes made to changelog (subsection may not exist)")
        sys.exit(0)


if __name__ == "__main__":
    main()
