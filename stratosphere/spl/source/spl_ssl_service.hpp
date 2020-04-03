/*
 * Copyright (c) 2018-2020 Atmosphère-NX
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once
#include "spl_rsa_service.hpp"

namespace ams::spl {

    class SslService : public RsaService {
        public:
            SslService() : RsaService() { /* ... */ }
            virtual ~SslService() { /* ... */ }
        protected:
            /* Actual commands. */
            virtual Result ImportSslKey(const sf::InPointerBuffer &src, AccessKey access_key, KeySource key_source);
            virtual Result SslExpMod(const sf::OutPointerBuffer &out, const sf::InPointerBuffer &base, const sf::InPointerBuffer &mod);
        public:
            DEFINE_SERVICE_DISPATCH_TABLE {
                MAKE_SERVICE_COMMAND_META(GetConfig),
                MAKE_SERVICE_COMMAND_META(ExpMod),
                MAKE_SERVICE_COMMAND_META(SetConfig),
                MAKE_SERVICE_COMMAND_META(GenerateRandomBytes),
                MAKE_SERVICE_COMMAND_META(IsDevelopment),
                MAKE_SERVICE_COMMAND_META(SetBootReason,                  hos::Version_300),
                MAKE_SERVICE_COMMAND_META(GetBootReason,                  hos::Version_300),
                MAKE_SERVICE_COMMAND_META(GenerateAesKek),
                MAKE_SERVICE_COMMAND_META(LoadAesKey),
                MAKE_SERVICE_COMMAND_META(GenerateAesKey),
                MAKE_SERVICE_COMMAND_META(DecryptAesKey),
                MAKE_SERVICE_COMMAND_META(CryptAesCtr),
                MAKE_SERVICE_COMMAND_META(ComputeCmac),
                MAKE_SERVICE_COMMAND_META(AllocateAesKeyslot,             hos::Version_200),
                MAKE_SERVICE_COMMAND_META(FreeAesKeyslot,                 hos::Version_200),
                MAKE_SERVICE_COMMAND_META(GetAesKeyslotAvailableEvent,    hos::Version_200),
                MAKE_SERVICE_COMMAND_META(DecryptRsaPrivateKeyDeprecated, hos::Version_400, hos::Version_400),
                MAKE_SERVICE_COMMAND_META(DecryptRsaPrivateKey,           hos::Version_500),
                MAKE_SERVICE_COMMAND_META(ImportSslKey,                   hos::Version_500),
                MAKE_SERVICE_COMMAND_META(SslExpMod,                      hos::Version_500),

            };
    };

}
