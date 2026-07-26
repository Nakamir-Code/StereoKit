# Print version, so we know we're building the right version right away
$fileData = Get-Content -path "$PSScriptRoot\..\StereoKitC\stereokit.h" -Raw;
$fileData -match '#define SK_VERSION_MAJOR\s+(?<ver>\d+)' | Out-Null
$major = $Matches.ver
$fileData -match '#define SK_VERSION_MINOR\s+(?<ver>\d+)' | Out-Null
$minor = $Matches.ver
$fileData -match '#define SK_VERSION_PATCH\s+(?<ver>\d+)' | Out-Null
$patch = $Matches.ver
$fileData -match '#define SK_VERSION_PRERELEASE\s+(?<ver>\d+)' | Out-Null
$pre = $Matches.ver

$str = "$major.$minor.$patch"
if ($pre -ne 0) {
    $str = "$str-preview.$pre"
}

$csprojPath = "$PSScriptRoot\..\StereoKit\StereoKit.csproj"
if (Test-Path $csprojPath) {
    $csprojData = Get-Content -Path $csprojPath -Raw
    if ($csprojData -match '<Version>(?<ver>[^<]+)</Version>') {
        $full = $Matches.ver.Trim()
        if (($full.Length -gt $str.Length) -and $full.StartsWith($str) -and
            (@('.', '-') -contains $full.Substring($str.Length, 1))) {
            $str = $full
        }
    }
}

return @{
    'str' = $str; 
    'major' = $major;
    'minor' = $minor;
    'patch' = $patch; 
    'pre' = $pre }