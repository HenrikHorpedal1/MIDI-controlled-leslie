# Fusion 360 script: Set Group Mass
#
# Select multiple bodies, enter their combined measured mass in grams.
# The script computes the required density (total_mass / total_volume) and
# applies a single custom material to all selected bodies so Fusion's physics
# reflect the real-world weight.
#
# Internal Fusion API units:
#   mass    = kg
#   length  = cm  (volume = cm^3)
#   density = kg/cm^3  (Fusion material property stores kg/m^3, converted on write)

import adsk.core
import adsk.fusion
import traceback

NAME_PREFIX = 'GROUP_MASS_'

_handlers = []


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        if not adsk.fusion.Design.cast(app.activeProduct):
            raise RuntimeError('Open a Fusion Design first.')

        cmd_id = 'SetGroupMass'
        existing = ui.commandDefinitions.itemById(cmd_id)
        if existing:
            existing.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            cmd_id,
            'Set Group Mass',
            'Select multiple bodies and enter their combined measured mass in grams.'
        )

        on_created = _CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        cmd_def.execute()
        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class _CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            sel = inputs.addSelectionInput(
                'bodies',
                'Bodies',
                'Select all bodies that make up the weighed group'
            )
            sel.addSelectionFilter('Bodies')
            sel.addSelectionFilter('Occurrences')
            sel.setSelectionLimits(1, 0)   # 1 minimum, 0 = unlimited

            inputs.addFloatSpinnerCommandInput(
                'mass_g',
                'Total measured mass (g)',
                '',
                0.001,
                1000000.0,
                0.1,
                1.0
            )

            inputs.addStringValueInput(
                'mat_label',
                'Material label (optional)',
                ''
            )

            on_execute = _ExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

            on_destroy = _DestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)

        except:
            adsk.core.Application.get().userInterface.messageBox(
                'Setup failed:\n{}'.format(traceback.format_exc()))


class _ExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = None
        try:
            app    = adsk.core.Application.get()
            ui     = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)

            cmd_inputs  = args.command.commandInputs
            sel_input   = cmd_inputs.itemById('bodies')
            mass_input  = cmd_inputs.itemById('mass_g')
            label_input = cmd_inputs.itemById('mat_label')

            mass_g  = mass_input.value
            mass_kg = mass_g / 1000.0
            label   = label_input.value.strip()

            # Collect all native bodies from the selection
            all_bodies = []
            source_ents = []
            for i in range(sel_input.selectionCount):
                ent = sel_input.selection(i).entity
                source_ents.append(ent)
                all_bodies.extend(_native_bodies(ent))
            all_bodies = _unique(all_bodies)

            if not all_bodies:
                raise RuntimeError('No valid bodies found in selection.')

            # Sum volumes across all bodies
            total_vol_cm3 = 0.0
            for body in all_bodies:
                phys = body.getPhysicalProperties(
                    adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                vol = phys.mass / phys.density   # cm^3 (mass kg, density kg/cm^3)
                if vol < 1e-20:
                    raise RuntimeError(
                        f'Could not determine volume for body "{body.name}". '
                        'Assign any physical material to it first.')
                total_vol_cm3 += vol

            density_kg_cm3 = mass_kg / total_vol_cm3

            # Build material name
            if label:
                base_name = NAME_PREFIX + _safe_name(label)
            else:
                base_name = NAME_PREFIX + f'{len(all_bodies)}bodies'
            mat_name = _unique_material_name(design, base_name)

            # Copy a base material and set the density
            base = _base_material(all_bodies[0], design)
            mat  = design.materials.addByCopy(base, mat_name)
            if not mat:
                raise RuntimeError(f'Failed to create material "{mat_name}".')

            _set_density(mat, density_kg_cm3)

            # Apply to every body
            for body in all_bodies:
                body.material = mat

            # Verify by re-reading total mass
            actual_mass_kg = 0.0
            for body in all_bodies:
                phys = body.getPhysicalProperties(
                    adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                actual_mass_kg += phys.mass
            actual_g = actual_mass_kg * 1000.0

            body_names = ', '.join(b.name for b in all_bodies)
            ui.messageBox(
                f'Material "{mat_name}" applied to {len(all_bodies)} body/bodies.\n\n'
                f'Bodies       : {body_names}\n'
                f'Target mass  : {mass_g:.4f} g\n'
                f'Actual mass  : {actual_g:.4f} g\n'
                f'Total volume : {total_vol_cm3:.4f} cm³\n'
                f'Density set  : {density_kg_cm3 * 1000:.4f} g/cm³'
            )

        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class _DestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        _handlers.clear()
        adsk.terminate()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _unique_material_name(design, base_name):
    if not design.materials.itemByName(base_name):
        return base_name
    i = 2
    while True:
        candidate = f'{base_name}_v{i}'
        if not design.materials.itemByName(candidate):
            return candidate
        i += 1


def _native_bodies(ent):
    occ = adsk.fusion.Occurrence.cast(ent)
    if occ:
        out = []
        for proxy in occ.bRepBodies:
            b = proxy.nativeObject if proxy.nativeObject else proxy
            out.append(b)
        return out
    body = adsk.fusion.BRepBody.cast(ent)
    if body:
        native = body.nativeObject if body.nativeObject else body
        return [native]
    return []


def _unique(bodies):
    seen, out = set(), []
    for b in bodies:
        tok = b.entityToken
        if tok not in seen:
            seen.add(tok)
            out.append(b)
    return out


def _base_material(body, design):
    if body.material:
        return body.material
    if design.materials.count > 0:
        return design.materials.item(0)
    raise RuntimeError('No material available to copy from. Assign any material to a body first.')


def _set_density(material, density_kg_cm3):
    """Write density to the material. Fusion stores it in kg/m³ internally."""
    value_kg_m3 = density_kg_cm3 * 1e6

    def _try_coll(coll):
        count = getattr(coll, 'count', None)
        if count is None:
            return False
        for i in range(count):
            try:
                p = coll.item(i)
            except Exception:
                continue
            pname = getattr(p, 'name', '')
            pid   = getattr(p, 'id',   '')
            if 'dens' in f'{pname} {pid}'.lower() and not getattr(p, 'isReadOnly', False):
                fp = adsk.core.FloatProperty.cast(p)
                if fp:
                    fp.value = value_kg_m3
                    return True
                if hasattr(p, 'value'):
                    p.value = value_kg_m3
                    return True
        return False

    pg = getattr(material, 'propertyGroups', None)
    if pg is not None:
        for gi in range(pg.count):
            g      = pg.item(gi)
            nested = getattr(g, 'properties', None)
            if nested is not None and _try_coll(nested):
                return
            if _try_coll(g):
                return

    mp = getattr(material, 'properties', None)
    if mp is not None and _try_coll(mp):
        return

    mpp = getattr(material, 'materialProperties', None)
    if mpp is not None and _try_coll(mpp):
        return

    raise RuntimeError('Could not find a writable Density property on the material.')


def _safe_name(s):
    bad = '<>:"/\\|?*'
    out = ''.join('_' if c in bad else c for c in s)
    return out.strip() or 'Group'
