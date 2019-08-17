/*
 * Copyright (c) 2019 Atmosphère-NX
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

#include "vgic.h"
#include "irq.h"
#include "utils.h"
#include "core_ctx.h"

#include "debug_log.h"

#define MAX_NUM_INTERRUPTS  (512 - 32 + 32 * 4)

#define GICDOFF(field)      (offsetof(ArmGicV2Distributor, field))

typedef struct VirqState {
    u32 listPrev        : 10;
    u32 listNext        : 10;
    u8  priority        : 5;
    bool pending        : 1;
    bool active         : 1;
    bool stray          : 1;
    bool pendingLatch   : 1;
    u32 coreId          : 3; // up to 8 cores, but not implemented yet
} VirqState;

typedef struct VirqStateList {
    VirqState *first, *last;
    size_t size;
} VirqStateList;

// Note: we reset the GIC from wakeup-from-sleep, and expect the guest OS to save/restore state if needed
static VirqState TEMPORARY g_virqStates[MAX_NUM_INTERRUPTS] = { 0 };
static VirqStateList TEMPORARY g_virqPendingQueues[4] = { { NULL } };
static VirqStateList TEMPORARY g_virqStrayPendingQueue = { NULL };
static u8 TEMPORARY g_virqCurrentHighestPriorities[4] = { 0 };

static bool TEMPORARY g_virqIsDistributorEnabled = false;

static inline VirqState *vgicGetVirqState(u32 coreId, u16 id)
{
    if (id >= 32) {
        return &g_virqStates[id];
    } else {
        return &g_virqStates[512 - 32 + 32 * coreId + id];
    }
}

static inline VirqState *vgicGetNextQueuedVirqState(VirqState *cur)
{
    return &g_virqStates[cur->listNext];
}

static inline VirqState *vgicGetPrevQueuedVirqState(VirqState *cur)
{
    return &g_virqStates[cur->listPrev];
}

static inline VirqState *vgicGetQueueEnd(void)
{
    return &g_virqStates[MAX_NUM_INTERRUPTS];
}

static inline u32 vgicGetVirqStateIndex(VirqState *cur)
{
    return (u32)(cur - &g_virqStates[0]);
}

// Note: ordered by priority
static void vgicEnqueueVirqState(VirqStateList *list, VirqState *elem)
{
    u32 prio = elem->priority;
    VirqState *pos;

    ++list->size;
    // Empty list
    if (list->first == vgicGetQueueEnd()) {
        list->first = list->last = elem;
        return;
    }

    for (pos = list->first; pos != vgicGetQueueEnd(); pos = vgicGetNextQueuedVirqState(pos)) {
        // Lowest priority number is higher
        if (elem->priority <= pos->priority) {
            break;
        }
    }

    if (pos == vgicGetQueueEnd()) {
        // Insert after last
        pos = list->last;
        elem->listPrev = vgicGetVirqStateIndex(pos);
        elem->listNext = pos->listNext;
        pos->listNext = vgicGetVirqStateIndex(elem);
        list->last = elem;
    } else {
        // Otherwise, insert before
        u32 idx = vgicGetVirqStateIndex(elem);
        u32 posidx = vgicGetVirqStateIndex(pos);
        u32 previdx = pos->listPrev;

        elem->listNext = posidx;
        elem->listPrev = previdx;

        pos->listPrev = idx;

        if (pos == list->first) {
            list->first = elem;
        } else {
            VirqState *prev = vgicGetPrevQueuedVirqState(pos);
            prev->listNext = idx;
        }

    }
}

static void vgicDequeueVirqState(VirqStateList *list, VirqState *elem)
{
    u32 idx = vgicGetVirqStateIndex(elem);
    VirqState *prev = vgicGetPrevQueuedVirqState(elem);
    VirqState *next = vgicGetNextQueuedVirqState(elem);

    --list->size;
    if (prev != vgicGetQueueEnd()) {
        prev->listNext = elem->listNext;
    } else {
        list->first = next;
    }

    if (next != vgicGetQueueEnd()) {
        next->listPrev = elem->listPrev;
    } else {
        list->last = prev;
    }
}

static inline void vgicNotifyOtherCoreList(u32 coreList)
{
    coreList &= ~BIT(currentCoreCtx->coreId);
    generateSgiForList(ThermosphereSgi_VgicUpdate, coreList);
}

static inline bool vgicIsVirqEdgeTriggered(u16 id)
{
    return (g_irqManager.gic.gicd->icfgr[id / 16] & (2 << (id % 16))) != 0;
}

static inline bool vgicIsVirqEnabled(u16 id)
{
    return (g_irqManager.gic.gicd->isenabler[id / 32] & BIT(id % 32)) != 0;
}

static inline bool vgicIsVirqPending(VirqState *state)
{
    // In case we emulate ispendr in the future...
    return state->pendingLatch || (!vgicIsVirqEdgeTriggered(vgicGetVirqStateIndex(state)) && state->pending);
}

//////////////////////////////////////////////////

static void vgicSetDistributorControlRegister(u32 value)
{
    // We implement a virtual distributor/interface w/o security extensions.
    // Moreover, we forward all interrupts as Group 0 so that non-secure code that assumes GICv2
    // *with* security extensions (and thus all interrupts fw as group 1 there) still works (bit are in the same positions).

    // We don't implement Group 1 interrupts, either (so that's similar to GICv1).
    bool old = g_virqIsDistributorEnabled;
    g_virqIsDistributorEnabled = (value & 1) != 0;

    // Enable bit is actually just a global enable bit for all irq forwarding, other functions of the GICD aren't affected by it
    if (old != g_virqIsDistributorEnabled) {
        generateSgiForAllOthers(ThermosphereSgi_VgicUpdate);
    }
}

static inline u32 vgicGetDistributorControlRegister(void)
{
    return g_virqIsDistributorEnabled ? 1 : 0;
}

static inline u32 vgicGetDistributorTypeRegister(void)
{
    // See above comment.
    // Therefore, LSPI = 0, SecurityExtn = 0, rest = from physical distributor
    return g_irqManager.gic.gicd->typer & 0x7F;
}

static inline u32 vgicGetDistributorImplementerIdentificationRegister(void)
{
    u32 iidr =  ((u32)'A' << 24); // Product Id: Atmosphère (?)
    iidr |= 1 << 16;    // Major revision 1
    iidr |= 0 << 12;    // Minor revision 0
    iidr |= 0x43B;      // Implementer: Arm (value copied from physical GICD)
    return iidr;
}

static void vgicSetInterruptEnabledState(u16 id)
{
    if (id < 16 || !vgicIsVirqEnabled(id)) {
        // Nothing to do...
        // Also, ignore for SGIs
        return;
    }

    // Similar effects to setting the target list to non-0 when it was 0...
    VirqState *state = vgicGetVirqState(currentCoreCtx->coreId, id);
    if (vgicIsVirqPending(state)) {
        vgicNotifyOtherCoreList(g_irqManager.gic.gicd->itargetsr[id]);
    }
}

static void vgicClearInterruptEnabledState(u16 id)
{
    if (id < 16 || !vgicIsVirqEnabled(id)) {
        // Nothing to do...
        // Also, ignore for SGIs
        return;
    }

    // Similar effects to setting the target list to 0, we may need to notify the core
    // handling the interrupt if it's pending
    VirqState *state = vgicGetVirqState(currentCoreCtx->coreId, id);
    if (vgicIsVirqPending(state)) {
        vgicNotifyOtherCoreList(BIT(state->coreId));
    }

    g_irqManager.gic.gicd->isenabler[id / 32] &= ~BIT(id % 32);
}

static inline bool vgicGetInterruptEnabledState(u16 id)
{
    // SGIs are always enabled
    return id < 16 || vgicIsVirqEnabled(id);
}

static void vgicSetInterruptPriorityByte(u16 id, u8 priority)
{
    // 32 priority levels max, bits [7:3]
    priority >>= 3;
    priority &= 0x1F;

    VirqState *state = vgicGetVirqState(currentCoreCtx->coreId, id);
    state->priority = priority;
    if (!state->stray && g_virqCurrentHighestPriorities[state->coreId] < priority) {
        vgicNotifyOtherCoreList(BIT(state->coreId));
    }
}

static inline u8 vgicGetInterruptPriorityByte(u16 id)
{
    return vgicGetVirqState(currentCoreCtx->coreId, id)->priority << 3;
}

static void vgicSetInterruptTargets(u16 id, u8 coreList)
{
    // Ignored for SGIs and PPIs
    if (id < 32) {
        return;
    }

    // Interrupt not pending (inactive or active-only): nothing much to do (see reference manual)
    // Otherwise, we may need to migrate the interrupt.
    // In our model, while a physical interrupt can be pending on multiple core, we decide that a pending SPI
    // can only be pending on a single core

    // Note that we take into account that the interrupt may be disabled.
    VirqState *state = vgicGetVirqState(currentCoreCtx->coreId, id);
    if (vgicIsVirqPending(state)) {
        u8 oldList = g_irqManager.gic.gicd->itargetsr[id];
        if (oldList != 0 && ((oldList ^ coreList) & BIT(state->coreId))) {
            // We need to migrate the irq away from the core handling it (and it's possibly in its list regs...)
            vgicNotifyOtherCoreList(BIT(state->coreId));
            // other core will be waiting for lock to be unlocked here
        } else if (oldList != coreList && coreList != 0) {
            // Only case I can think of is when oldList was 0
            // Just leave it in the stray list, and notifiy the new potential targets
            vgicNotifyOtherCoreList(coreList);
            // other cores will be waiting for lock to be unlocked here
        }
    }
    g_irqManager.gic.gicd->itargetsr[id] = coreList;
}

static inline u8 vgicGetInterruptTargets(u16 id)
{
    // For SGIs & PPIs, itargetsr is banked and contains the CPU ID
    return g_irqManager.gic.gicd->itargetsr[id];
}

static inline void vgicSetInterruptConfigByte(u16 id, u32 config)
{
    // Ignored for SGIs, implementation defined for PPIs
    if (id < 32) {
        return;
    }

    u32 cfg = g_irqManager.gic.gicd->icfgr[id / 16];
    cfg &= ~(3 << (id % 16));
    cfg |= (config & 2) << (id % 16);
    g_irqManager.gic.gicd->icfgr[id / 16] = cfg;
}

static inline u32 vgicGetInterruptConfigByte(u16 id, u32 config)
{
    return vgicIsVirqEdgeTriggered(id) ? 2 : 0;
}

static void vgicSetSgiPendingState(u16 id, u32 coreId, u32 srcCoreId)
{
    u8 old = g_irqManager.sgiPendingSources[coreId][id];
    g_irqManager.sgiPendingSources[coreId][id] = old | srcCoreId;
    if (old == 0) {
        // SGI is now pending & possibly needs to be serviced
        VirqState *state = vgicGetVirqState(coreId, id);
        state->pendingLatch = true;
        state->coreId = coreId;
        vgicEnqueueVirqState(&g_virqPendingQueues[coreId], state);
        vgicNotifyOtherCoreList(BIT(coreId));
    }
}

static void vgicClearSgiPendingState(u16 id, u32 srcCoreId)
{
    // Only for the current core, therefore no need to signal physical SGI, etc., etc.
    u32 coreId = currentCoreCtx->coreId;
    u8 old = g_irqManager.sgiPendingSources[coreId][id];
    u8 new_ =  old & ~(u8)srcCoreId;
    g_irqManager.sgiPendingSources[coreId][id] = new_;
    if (old != 0 && new_ == 0) {
        VirqState *state = vgicGetVirqState(coreId, id);
        state->pendingLatch = false;
        // vgicDequeueVirqState(&g_virqPendingQueues[coreId], state); nope, since it may be in a LR instead
        vgicNotifyOtherCoreList(BIT(coreId));
    }
}

static inline u32 vgicGetSgiPendingState(u16 id)
{
    return g_irqManager.sgiPendingSources[currentCoreCtx->coreId][id];
}

static void vgicSendSgi(u16 id, u32 filter, u32 coreList)
{
    switch (filter) {
        case 0:
            // Forward to coreList
            break;
        case 1:
            // Forward to all but current core
            coreList = getActiveCoreMask() & ~BIT(currentCoreCtx->coreId);
            break;
        case 2:
            // Forward to current core only
            coreList = BIT(currentCoreCtx->coreId);
            break;
        default:
            DEBUG("Emulated GCID_SGIR: invalid TargetListFilter value!\n");
            return;
    }

    FOREACH_BIT(tmp, dstCore, coreList) {
        vgicSetSgiPendingState(id, dstCore, currentCoreCtx->coreId);
    }
}

static inline u32 vgicGetPeripheralId2Register(void)
{
    // 2 for Gicv2
    return 2 << 4;
}

static void handleVgicMmioWrite(ExceptionStackFrame *frame, DataAbortIss dabtIss, size_t offset)
{
    size_t sz = BITL(dabtIss.sas);
    u32 val = (u32)frame->x[dabtIss.srt];
    uintptr_t addr = (uintptr_t)g_irqManager.gic.gicd + offset;

    switch (offset) {
        case GICDOFF(typer):
        case GICDOFF(iidr):
        case GICDOFF(icpidr2):
        case GICDOFF(itargetsr) ... GICDOFF(itargetsr) + 31:
            // Write ignored (read-only registers)
            break;
        case GICDOFF(icfgr) ... GICDOFF(icfgr) + 31/4:
            // Write ignored because of an implementation-defined choice
            break;
        case GICDOFF(igroupr) ... GICDOFF(igroupr) + 511/32:
            // Write ignored because we don't implement Group 1 here
            break;
        case GICDOFF(ispendr) ... GICDOFF(ispendr) + 511/32:
        case GICDOFF(icpendr) ... GICDOFF(icpendr) + 511/32:
        case GICDOFF(isactiver) ... GICDOFF(isactiver) + 511/32:
        case GICDOFF(icactiver) ... GICDOFF(icactiver) + 511/32:
            // Write ignored, not implemented (at least not yet, TODO)
            break;

        case GICDOFF(ctlr):
            vgicSetDistributorControlRegister(val);
            break;

        case GICDOFF(isenabler) ... GICDOFF(isenabler) + 511/32: {
            u32 base = 32 * (offset - GICDOFF(isenabler));
            FOREACH_BIT(tmp, pos, val) {
                vgicSetInterruptEnabledState((u16)(base + pos));
            }
            break;
        }
        case GICDOFF(icenabler) ... GICDOFF(icenabler) + 511/32: {
            u32 base = 32 * (offset - GICDOFF(icenabler));
            FOREACH_BIT(tmp, pos, val) {
                vgicClearInterruptEnabledState((u16)(base + pos));
            }
            break;
        }

        case GICDOFF(ipriorityr) ... GICDOFF(ipriorityr) + 511: {
            u16 base = (u16)(offset - GICDOFF(ipriorityr));
            for (u16 i = 0; i < sz; i++) {
                vgicSetInterruptPriorityByte(base + i, (u8)val);
                val >>= 8;
            }
            break;
        }

        case GICDOFF(itargetsr) + 32 ... GICDOFF(itargetsr) + 511: {
            u16 base = (u16)(offset - GICDOFF(itargetsr));
            for (u16 i = 0; i < sz; i++) {
                vgicSetInterruptTargets(base + i, (u8)val);
                val >>= 8;
            }
            break;
        }

        case GICDOFF(sgir):
            vgicSendSgi((u16)(val & 0xF), (val >> 24) & 3, (val >> 16) & 0xFF);
            break;

        case GICDOFF(cpendsgir) ... GICDOFF(cpendsgir) + 15: {
            u16 base = (u16)(offset - GICDOFF(cpendsgir));
            for (u16 i = 0; i < sz; i++) {
                FOREACH_BIT(tmp, pos, val & 0xFF) {
                    vgicClearSgiPendingState(base + i, pos);
                }
                val >>= 8;
            }
            break;
        }
        case GICDOFF(spendsgir) ... GICDOFF(spendsgir) + 15: {
            u16 base = (u16)(offset - GICDOFF(spendsgir));
            for (u16 i = 0; i < sz; i++) {
                FOREACH_BIT(tmp, pos, val & 0xFF) {
                    vgicSetSgiPendingState(base + i, currentCoreCtx->coreId, pos);
                }
                val >>= 8;
            }
            break;
        }

        default:
            dumpUnhandledDataAbort(dabtIss, addr, "GICD reserved/implementation-defined register");
            break;
    }
}


static void handleVgicMmioRead(ExceptionStackFrame *frame, DataAbortIss dabtIss, size_t offset)
{
    size_t sz = BITL(dabtIss.sas);
    uintptr_t addr = (uintptr_t)g_irqManager.gic.gicd + offset;

    u32 val = 0;

    switch (offset) {
        case GICDOFF(icfgr) ... GICDOFF(icfgr) + 31/4:
            // RAZ because of an implementation-defined choice
            break;
        case GICDOFF(igroupr) ... GICDOFF(igroupr) + 511/32:
            // RAZ because we don't implement Group 1 here
            break;
        case GICDOFF(ispendr) ... GICDOFF(ispendr) + 511/32:
        case GICDOFF(icpendr) ... GICDOFF(icpendr) + 511/32:
        case GICDOFF(isactiver) ... GICDOFF(isactiver) + 511/32:
        case GICDOFF(icactiver) ... GICDOFF(icactiver) + 511/32:
            // RAZ, not implemented (at least not yet, TODO)
            break;

        case GICDOFF(ctlr):
            val = vgicGetDistributorControlRegister();
            break;
        case GICDOFF(typer):
            val = vgicGetDistributorTypeRegister();
            break;
        case GICDOFF(iidr):
            val = vgicGetDistributorImplementerIdentificationRegister();
            break;

        case GICDOFF(isenabler) ... GICDOFF(isenabler) + 511/32:
        case GICDOFF(icenabler) ... GICDOFF(icenabler) + 511/32: {
            u16 base = (u16)(32 * (offset & 0x7F));
            for (u16 i = 0; i < 32; i++) {
                val |= vgicGetInterruptEnabledState(base + i) ? BIT(i) : 0;
            }
            break;
        }

        case GICDOFF(ipriorityr) ... GICDOFF(ipriorityr) + 511: {
            u16 base = (u16)(offset - GICDOFF(ipriorityr));
            for (u16 i = 0; i < sz; i++) {
                val |= vgicGetInterruptPriorityByte(base + i) << (8 * i);
            }
            break;
        }

        case GICDOFF(itargetsr) ... GICDOFF(itargetsr) + 511: {
            u16 base = (u16)(offset - GICDOFF(itargetsr));
            for (u16 i = 0; i < sz; i++) {
                val |= (u32)vgicGetInterruptTargets(base + i) << (8 * i);
            }
            break;
        }

        case GICDOFF(sgir):
            // Write-only register
            val = 0xD15EA5E5;
            break;

        case GICDOFF(cpendsgir) ... GICDOFF(cpendsgir) + 15:
        case GICDOFF(spendsgir) ... GICDOFF(spendsgir) + 15: {
            u16 base = (u16)(offset & 0xF);
            for (u16 i = 0; i < sz; i++) {
                val |= (u32)vgicGetSgiPendingState(base + i) << (8 * i);
            }
            break;
        }

        case GICDOFF(icpidr2):
            val = vgicGetPeripheralId2Register();
            break;

        default:
            dumpUnhandledDataAbort(dabtIss, addr, "GICD reserved/implementation-defined register");
            break;
    }

    frame->x[dabtIss.srt] = val;
}

void handleVgicdMmio(ExceptionStackFrame *frame, DataAbortIss dabtIss, size_t offset)
{
    size_t sz = BITL(dabtIss.sas);
    uintptr_t addr = (uintptr_t)g_irqManager.gic.gicd + offset;

    // ipriorityr, itargetsr, *pendsgir are byte-accessible
    if (
        !(offset >= GICDOFF(ipriorityr) && offset < GICDOFF(ipriorityr) + 512) &&
        !(offset >= GICDOFF(itargetsr)  && offset < GICDOFF(itargetsr) + 512) && 
        !(offset >= GICDOFF(cpendsgir)  && offset < GICDOFF(cpendsgir) + 16) && 
        !(offset >= GICDOFF(spendsgir)  && offset < GICDOFF(spendsgir) + 16)
    ) {
        if ((offset & 3) != 0 || sz != 4) {
            dumpUnhandledDataAbort(dabtIss, addr, "GICD non-word aligned MMIO");
        }
    } else {
        if (sz != 1 && sz != 4) {
            dumpUnhandledDataAbort(dabtIss, addr, "GICD 16 or 64-bit access");
        } else if (sz == 4 && (offset & 3) != 0) {
            dumpUnhandledDataAbort(dabtIss, addr, "GICD misaligned MMIO");
        }
    }

    recursiveSpinlockLock(&g_irqManager.lock);

    if (dabtIss.wnr) {
        handleVgicMmioWrite(frame, dabtIss, offset);
    } else {
        handleVgicMmioRead(frame, dabtIss, offset);
    }

    // TODO gic main loop
    recursiveSpinlockUnlock(&g_irqManager.lock);
}

#if 0
// TODO: this is modeled after kvm, but I think it's full of race conditions

static void vgicSetInterruptPendingState(u16 id)
{
    // Note: don't call this for SGIs
    VirqState *state = vgicGetVirqState(currentCoreCtx->coreId, id);
    bool wasPending = vgicIsVirqPending(state);
    state->pendingLatch = true;

    // Since this is a (forwarded) hardware interrupt, we need to set it active in the physical distributor
    g_irqManager.gic.gicd->isactiver[id / 32] |= BIT(id % 32);

    if(!wasPending) {
        // Interrupt wasn't pending. We need to add it to a queue (select the first target)
        u32 targets = g_irqManager.gic.gicd->itargetsr[id];
        u32 firstTarget = __builtin_ffs(targets);
        if (firstTarget != 0) {
            vgicEnqueueVirqState(&g_virqPendingQueues[firstTarget - 1], state);
            generateSgiForList(ThermosphereSgi_VgicUpdate, BIT(firstTarget - 1));
        }
    }
}

static void vgicClearInterruptPendingState(u16 id)
{
    // Note: don't call this for SGIs
    VirqState *state = vgicGetVirqState(currentCoreCtx->coreId, id);
    bool wasPending = vgicIsVirqPending(state);
    state->pendingLatch = false;

    /*
        Since this is a (forwarded) hardware interrupt...:
            - clear physical pending state, to ensure proper functioning of edge-triggered interrupts
            - clear physical active pending state, but only if the virtual interrupt itself isn't active
                - at worst, this will only cause us to receive the interrupt once more
     */
    // TODO check if state->active reflects the state of the interrupt
    if (state->edgeTriggered) {
        g_irqManager.gic.gicd->icpendr[id / 32] |= BIT(id % 32);
    }

    if (!state->active) {
        g_irqManager.gic.gicd->icactiver[id / 32] |= BIT(id % 32);
    }
}
#endif

void vgicInit(void)
{
    if (currentCoreCtx->isBootCore) {
        for (u32 i = 0; i < 4; i++) {
            g_virqPendingQueues[i].first = g_virqPendingQueues[i].last = vgicGetQueueEnd();
            g_virqCurrentHighestPriorities[i] = 0x1F; // 32 priority level max on virtual interface
            // SGIs are edge-triggered
            /*for (u32 j = 0; j < 16; j++) {
                vgicGetVirqState(i, (u16)j)->edgeTriggered = true;
            }*/
        }

        for (u32 i = 0; i < 512 - 32 + 32 * 4; i++) {
            g_virqStates[i].listNext = g_virqStates[i].listPrev = MAX_NUM_INTERRUPTS;
        }
    }
}

// lock needs to be held by caller
void vgicEnqueuePhysicalIrq(u16 id)
{

}