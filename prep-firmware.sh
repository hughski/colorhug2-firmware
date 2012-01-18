checksum=`sha1sum $1 | cut -d" " -f1`
size=`du --bytes $1 | cut -f1`
timestamp=`du --time --time-style=+%s $1 | cut -f2`
version=`echo $1 | cut -f2`

echo "  <update>"
echo "    <state>stable</state>"
echo "    <version>FIXME$1</version>"
echo "    <supported_hardware>1</supported_hardware>"
echo "    <checksum type=\"sha1\">$checksum</checksum>"
echo "    <timestamp>$timestamp</timestamp>"
echo "    <filename>$1</filename>"
echo "    <size>$size</size>"
echo "    <changelog>"
echo "     <item>"
echo "      FIXME"
echo "     </item>"
echo "    </changelog>"
echo "  </update>"
