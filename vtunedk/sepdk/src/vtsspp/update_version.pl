#!/usr/bin/perl
###############################################################################
#
# Copyright (C) 2012 Intel Corporation.  All Rights Reserved.
#
# This file is part of SEP Development Kit
#
# SEP Development Kit is free software; you can redistribute it
# and/or modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation.
#
# SEP Development Kit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SEP Development Kit; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
#
# As a special exception, you may use this file as part of a free software
# library without restriction.  Specifically, if other files instantiate
# templates or use macros or inline functions from this file, or you compile
# this file and link it with other files to produce an executable, this
# file does not by itself cause the resulting executable to be covered by
# the GNU General Public License.  This exception does not however
# invalidate any other reasons why the executable file might be covered by
# the GNU General Public License.
#
###############################################################################

($curr_linux_ascii, $infile, $outfile) = @ARGV;
die "Missing input file name.\n" unless $infile;
die "Missing output file name.\n" unless $outfile;

$| = 1;

#
# using conversion functions written by John Winger found at planet-source-code.com
#
sub ascii_to_hex ($)
{
    ## Convert each ASCII character to a two-digit hex number.
    (my $str = shift) =~ s/(.|\n)/sprintf("%02lx", ord $1)/eg;
    return $str;
}

sub hex_to_ascii ($)
{
    ## Convert each two-digit hex number back to an ASCII character.
    (my $str = shift) =~ s/([a-fA-F0-9]{2})/chr(hex $1)/eg;
    return $str;
}

# 'vermagic='
@vermagic_str = ("76","65","72","6d","61","67","69","63","3d");
# ' SMP mod_unload '
@SMP_str = ("20","53","4d","50","20", "6d", "6f", "64", "5f", "75", "6e", "6c", "6f", "61", "64", "20");

$byteCount = 0;
open(IN, "< $infile") or die ("ERROR: Could not open $infile");
binmode(IN);
open(OUT, "> $outfile");
binmode(OUT);

# get linux version on current system
chomp($curr_linux_ascii);

my($hex);
my($out);
my($found) = 0;

while (read(IN,$b,1)) {
    $byteCount++;
    $hex = unpack( 'H*', $b );

    if ($found == 1) {
        $out = pack( 'H*', $hex );   
        print(OUT $out);
        next;
    }

    # parse through the 'vermagic=' string
    if ($hex eq $vermagic_str[0]) {
        $out = pack( 'H*', $hex );
        print(OUT $out);

        for ($i = 1; $i < 9; $i++) {
            read(IN,$b,1);
            $byteCount++;

            # want this all transfered to the new binary
            $hex = unpack( 'H*', $b );

            if ($hex ne $vermagic_str[$i]) {
                last;
            }

            $out = pack( 'H*', $hex);
            print(OUT $out);

            if ($i == 8) {
                $found = 1;
            }
        }

        # extract linux version in driver
        $drv_linux = '';
        while (($found == 1) &&
               ($hex ne $SMP_str[0])) {
            read(IN,$b,1);
            $hex = unpack( 'H*', $b );
            $byteCount++;

            if ($hex eq $SMP_str[0]) {
                last;
            }

            # save the driver's version of linux for comparison
            $drv_linux_hex = $drv_linux_hex . $hex;
        }
        $drv_linux_ascii = hex_to_ascii($drv_linux_hex);

        # compare the two and see if they're the same
        if ($drv_linux_ascii eq $curr_linux_ascii) {
            print "Current linux installation matches the driver version\n";

            close(IN);
            close(OUT);

            #delete new output file.
            #unlink($outfile);
            exit;
        }

        # calculate if there is any difference in size between the different linux 
        # strings.  We need to maintain the same file size.
        if ($found == 1) {
            use bytes;

            $drv_linux_size = length($drv_linux_ascii);
            $curr_linux_size = length($curr_linux_ascii);
            $size_diff = $curr_linux_size - $drv_linux_size;

            # compatible binaries?
            # major, minor, revision numbers need to match
            # driver versioning
            my($drv_major);
            my($drv_minor);
            my($drv_rev);
            $drv_linux_ascii =~ /(\d+)\.(\d+)\.(\d+)?[\D].*/;
            $drv_major = $1;
            $drv_minor = $2;
            $drv_rev = $3;

            # linux installation versioning
            my($curr_major);
            my($curr_minor);
            my($curr_rev);
            $curr_linux_ascii =~ /(\d+)\.(\d+)\.(\d+)?[\D].*/;
            $curr_major = $1;
            $curr_minor = $2;
            $curr_rev = $3;

            if (($drv_major ne $curr_major) ||
                ($drv_minor ne $curr_minor) ||
                ($drv_rev ne $curr_rev)) {
              # we have an incompatibility
              print "ERROR: cannot update $infile to accomodate linux version $curr_linux_ascii!\n";

              close(IN);
              close(OUT);

              #delete new output file
              unlink($outfile);
              exit;
            }

            # replace driver linux version with current linux installation string and continue
            $out = pack( 'H*', ascii_to_hex($curr_linux_ascii));
            print(OUT $out);
            # need to overwrite the " SMP mod_unload " string in case the new linux name and the
            # old linux names are not the same length
            for ($j = 0; $j < 16; $j++) {
                $out = pack('H*', $SMP_str[$j]);
                print(OUT $out);
                $byteCount++;
            }
            # also move the pointer in the input file ahead past
            # the SMP string
            read(IN,$b,15);

            # now we need to drop the additional bytes that were added in because of the longer
            # driver name
            if ($size_diff > 0) {
                read(IN,$b,$size_diff);
            }
        }
      }

    if ($found == 0) {
        $out = pack( 'H*', $hex );   
        print(OUT $out);
    }
}
close(IN);
close(OUT);

# delete original file
unlink($infile);

print "Updating $infile to $outfile\n";
exit;
