/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn/J2735_201603DA.ASN"
 * 	`asn1c -fcompound-names -pdu=all`
 */

#include "VehicleIdent.h"

static int check_permitted_alphabet_4(const void *sptr) {
	/* The underlying type is IA5String */
	const IA5String_t *st = (const IA5String_t *)sptr;
	const uint8_t *ch = st->buf;
	const uint8_t *end = ch + st->size;
	
	for(; ch < end; ch++) {
		uint8_t cv = *ch;
		if(!(cv <= 127)) return -1;
	}
	return 0;
}

static int
memb_ownerCode_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const IA5String_t *st = (const IA5String_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	size = st->size;
	
	if((size >= 1 && size <= 32)
		 && !check_permitted_alphabet_4(st)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_vehicleClass_constr_7 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_vehicleClass_constr_7 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_ownerCode_constr_4 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..32)) */};
static asn_per_constraints_t asn_PER_memb_ownerCode_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 7,  7,  0,  127 }	/* (0..127) */,
	{ APC_CONSTRAINED,	 5,  5,  1,  32 }	/* (SIZE(1..32)) */,
	0, 0	/* No PER character map necessary */
};
static asn_TYPE_member_t asn_MBR_vehicleClass_7[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleIdent__vehicleClass, choice.vGroup),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleGroupAffected,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vGroup"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleIdent__vehicleClass, choice.rGroup),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ResponderGroupAffected,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"rGroup"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleIdent__vehicleClass, choice.rEquip),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IncidentResponseEquipment,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"rEquip"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_vehicleClass_tag2el_7[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* vGroup */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* rGroup */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* rEquip */
};
static asn_CHOICE_specifics_t asn_SPC_vehicleClass_specs_7 = {
	sizeof(struct VehicleIdent__vehicleClass),
	offsetof(struct VehicleIdent__vehicleClass, _asn_ctx),
	offsetof(struct VehicleIdent__vehicleClass, present),
	sizeof(((struct VehicleIdent__vehicleClass *)0)->present),
	asn_MAP_vehicleClass_tag2el_7,
	3,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_vehicleClass_7 = {
	"vehicleClass",
	"vehicleClass",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_vehicleClass_constr_7, &asn_PER_type_vehicleClass_constr_7, CHOICE_constraint },
	asn_MBR_vehicleClass_7,
	3,	/* Elements count */
	&asn_SPC_vehicleClass_specs_7	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_VehicleIdent_1[] = {
	{ ATF_POINTER, 6, offsetof(struct VehicleIdent, name),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DescriptiveName,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"name"
		},
	{ ATF_POINTER, 5, offsetof(struct VehicleIdent, vin),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VINstring,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vin"
		},
	{ ATF_POINTER, 4, offsetof(struct VehicleIdent, ownerCode),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IA5String,
		0,
		{ &asn_OER_memb_ownerCode_constr_4, &asn_PER_memb_ownerCode_constr_4,  memb_ownerCode_constraint_1 },
		0, 0, /* No default value */
		"ownerCode"
		},
	{ ATF_POINTER, 3, offsetof(struct VehicleIdent, id),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_VehicleID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
	{ ATF_POINTER, 2, offsetof(struct VehicleIdent, vehicleType),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vehicleType"
		},
	{ ATF_POINTER, 1, offsetof(struct VehicleIdent, vehicleClass),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_vehicleClass_7,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vehicleClass"
		},
};
static const int asn_MAP_VehicleIdent_oms_1[] = { 0, 1, 2, 3, 4, 5 };
static const ber_tlv_tag_t asn_DEF_VehicleIdent_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleIdent_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* name */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* vin */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* ownerCode */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* vehicleType */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 } /* vehicleClass */
};
asn_SEQUENCE_specifics_t asn_SPC_VehicleIdent_specs_1 = {
	sizeof(struct VehicleIdent),
	offsetof(struct VehicleIdent, _asn_ctx),
	asn_MAP_VehicleIdent_tag2el_1,
	6,	/* Count of tags in the map */
	asn_MAP_VehicleIdent_oms_1,	/* Optional members */
	6, 0,	/* Root/Additions */
	6,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_VehicleIdent = {
	"VehicleIdent",
	"VehicleIdent",
	&asn_OP_SEQUENCE,
	asn_DEF_VehicleIdent_tags_1,
	sizeof(asn_DEF_VehicleIdent_tags_1)
		/sizeof(asn_DEF_VehicleIdent_tags_1[0]), /* 1 */
	asn_DEF_VehicleIdent_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleIdent_tags_1)
		/sizeof(asn_DEF_VehicleIdent_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_VehicleIdent_1,
	6,	/* Elements count */
	&asn_SPC_VehicleIdent_specs_1	/* Additional specs */
};

