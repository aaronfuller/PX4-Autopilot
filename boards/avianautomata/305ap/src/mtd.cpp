#include <px4_platform_common/px4_manifest.h>

static const px4_mft_s mft = {
	.nmft = 0,
	.mfts = {}
};

const px4_mft_s *board_get_manifest(void)
{
	return &mft;
}
