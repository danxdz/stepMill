def extract_toolpath_block(text: str):
    """
    Extract the G-code toolpath block between toolpath markers.
    Returns: block_lines, start_idx, end_idx
    """
    lines = text.splitlines()
    start_idx, end_idx = None, None

    for i, line in enumerate(lines):
        if "; -- BEGIN TOOLPATH --" in line:
            start_idx = i
        elif "; -- END TOOLPATH --" in line:
            end_idx = i

    if start_idx is not None and end_idx is not None and start_idx < end_idx:
        return lines[start_idx + 1:end_idx], start_idx, end_idx
    return None, None, None


def replace_toolpath_block(original_text: str, new_toolpath_lines: list[str]) -> str:
    """
    Replace only the toolpath portion of the full G-code text.
    If no markers exist, add them at the end.
    """
    lines = original_text.splitlines()
    _, start_idx, end_idx = extract_toolpath_block(original_text)

    if start_idx is not None and end_idx is not None:
        return "\n".join(
            lines[:start_idx + 1] +
            new_toolpath_lines +
            lines[end_idx:]
        )
    else:
        return "\n".join(
            lines +
            ["", "; -- BEGIN TOOLPATH --"] +
            new_toolpath_lines +
            ["; -- END TOOLPATH --"]
        )
